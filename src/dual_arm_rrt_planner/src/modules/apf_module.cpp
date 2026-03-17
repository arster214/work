#include "dual_arm_rrt_planner/modules/apf_module.h"
#include "dual_arm_rrt_planner/dual_arm_rrt.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace dual_arm_planner
{
namespace modules
{

APFModule::APFModule()
    : attractive_gain_(1.0)
    , repulsive_gain_(5.0)
    , obstacle_influence_distance_(0.3)
    , decay_rate_(0.1)
    , max_force_magnitude_(0.5)
    , repulsive_force_threshold_(0.5)
    , near_field_distance_(0.05)
    , near_field_exponential_gain_(4.0)
{
}

bool APFModule::initialize(const ros::NodeHandle& nh, const std::string& name)
{
    nh_ = nh;
    name_ = name;
    nh_.param("apf_attractive_gain", attractive_gain_, 1.0);
    nh_.param("apf_repulsive_gain", repulsive_gain_, 5.0);
    nh_.param("apf_obstacle_influence_distance", obstacle_influence_distance_, 0.3);
    nh_.param("apf_decay_rate", decay_rate_, 0.1);
    nh_.param("apf_max_force", max_force_magnitude_, 0.5);
    nh_.param("apf_repulsive_force_threshold", repulsive_force_threshold_, 0.5);
    nh_.param("apf_near_field_distance", near_field_distance_, 0.05);
    nh_.param("apf_near_field_exponential_gain", near_field_exponential_gain_, 4.0);
    if (attractive_gain_ < 0.0) attractive_gain_ = std::abs(attractive_gain_);
    if (repulsive_gain_ < 0.0) repulsive_gain_ = std::abs(repulsive_gain_);
    if (obstacle_influence_distance_ <= 0.0) obstacle_influence_distance_ = 0.3;
    if (max_force_magnitude_ <= 0.0) max_force_magnitude_ = 0.5;
    ROS_INFO("[%s] APF Initialized: K_att=%.3f, K_rep=%.3f, rho0=%.3f", name_.c_str(), attractive_gain_, repulsive_gain_, obstacle_influence_distance_);
    return true;
}

void APFModule::postExtend(PlanningContext& context)
{
    if (!context.new_node_candidate)
    {
        ROS_WARN_THROTTLE(10.0, "[%s] Missing new_node_candidate", name_.c_str());
        return;
    }

    // 左臂受力计算
    Eigen::Vector3d force_left = computeTaskSpaceForce(context.current_task_point_left, context.target_task_point_left, context.obstacle_info_left);
    double force_mag_left = force_left.norm();
    if (force_mag_left > max_force_magnitude_) force_left = force_left * (max_force_magnitude_ / force_mag_left);
    context.modified_task_point_left = applyForce(context.current_task_point_left, force_left, context.step_size);

    // 右臂受力计算
    Eigen::Vector3d force_right = computeTaskSpaceForce(context.current_task_point_right, context.target_task_point_right, context.obstacle_info_right);
    double force_mag_right = force_right.norm();
    if (force_mag_right > max_force_magnitude_) force_right = force_right * (max_force_magnitude_ / force_mag_right);
    context.modified_task_point_right = applyForce(context.current_task_point_right, force_right, context.step_size);

    // 为了兼容旧逻辑，我们可选把力向量存入 apf_task_force（例如只需其中一个或者均值，暂存左臂）
    context.apf_task_force = force_left; 
    
    ROS_DEBUG_THROTTLE(10.0, "[%s] Computed APF. Left Force=%.4f, Right Force=%.4f", 
                       name_.c_str(), force_mag_left, force_mag_right);
}

Eigen::Vector3d APFModule::computeTaskSpaceForce(const Eigen::Vector3d& current_pos, 
                                                 const Eigen::Vector3d& target_pos, 
                                                 const ObstacleInfo& obs_info) const
{
    return calculateAttractiveForce(current_pos, target_pos) + calculateRepulsiveForce(obs_info);
}

Eigen::Vector3d APFModule::calculateAttractiveForce(const Eigen::Vector3d& current, const Eigen::Vector3d& target) const
{
    Eigen::Vector3d diff = target - current;
    if (diff.norm() < 1e-9) return Eigen::Vector3d::Zero();
    return attractive_gain_ * diff;
}

Eigen::Vector3d APFModule::calculateRepulsiveForce(const ObstacleInfo& obs_info) const
{
    if (!std::isfinite(obs_info.min_obstacle_distance) ||
        obs_info.min_obstacle_distance > obstacle_influence_distance_)
    {
        return Eigen::Vector3d::Zero();
    }

    const double d = std::max(obs_info.min_obstacle_distance, 1e-6);
    Eigen::Vector3d gradient = obs_info.closest_gradient;
    if (gradient.norm() < 1e-9)
        return Eigen::Vector3d::Zero();

    gradient.normalize();
    double magnitude = 0.0;
    if (d <= near_field_distance_)
    {
        magnitude = repulsive_gain_ * std::exp(near_field_exponential_gain_ * (near_field_distance_ - d));
    }
    else
    {
        const double rho = std::max(obstacle_influence_distance_, d + 1e-6);
        magnitude = repulsive_gain_ * (1.0 / d - 1.0 / rho) / (d * d);
    }
    if (d < repulsive_force_threshold_)
        magnitude *= 1.5;
    return magnitude * gradient;
}

std::pair<double, Eigen::Vector3d> APFModule::distanceAndGradientToAABB(const Eigen::Vector3d& point, const AABB& box) const
{
    Eigen::Vector3d clamped = point.cwiseMax(box.min).cwiseMin(box.max);
    Eigen::Vector3d delta = point - clamped;
    double dist = delta.norm();
    if (dist > 1e-9)
        return {dist, delta / dist};

    Eigen::Vector3d distances_to_faces;
    distances_to_faces.x() = std::min(point.x() - box.min.x(), box.max.x() - point.x());
    distances_to_faces.y() = std::min(point.y() - box.min.y(), box.max.y() - point.y());
    distances_to_faces.z() = std::min(point.z() - box.min.z(), box.max.z() - point.z());
    int axis = 0;
    distances_to_faces.minCoeff(&axis);
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    gradient(axis) = (point(axis) - 0.5 * (box.min(axis) + box.max(axis)) >= 0.0) ? 1.0 : -1.0;
    return {0.0, gradient};
}

// 保留旧接口实现以兼容 (虽已被改写，但为了明确移除旧声明的无用代码，这里显式删除)
Eigen::Vector3d APFModule::computeTaskSpaceForce(const PlanningContext& context) const
{
    // Redirect to new implementation based on context being mainly for left arm or right arm?
    // Old interface is ambiguous for dual arm. We assume it's not used or we map to left arm for fallback.
    return computeTaskSpaceForce(context.current_task_point_left, context.target_task_point_left, context.obstacle_info_left);
}

Eigen::Vector3d APFModule::applyForce(const Eigen::Vector3d& current_task_point, const Eigen::Vector3d& force, double step_size) const
{
    double scale = step_size * 0.02;
    return current_task_point + force * scale;
}

} // namespace modules
} // namespace dual_arm_planner

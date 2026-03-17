#include "improved_rrt_planner/modules/apf_module.h"
#include "improved_rrt_planner/improved_rrt.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace improved_rrt_planner
{
namespace modules
{

APFModule::APFModule()
    : attractive_gain_(1.0)
    , repulsive_gain_(5.0)
    , obstacle_influence_distance_(0.3)
    , decay_rate_(0.1)
    , max_force_magnitude_(0.5)
    , use_repulsive_force_(false)
    , use_numerical_gradient_(true)
    , gradient_delta_(0.01)
    , repulsive_force_threshold_(0.5)
{
}

bool APFModule::initialize(const ros::NodeHandle& nh, const std::string& name)
{
    nh_ = nh;
    name_ = name;
    
    // 读取基本参数
    nh_.param("apf_attractive_gain", attractive_gain_, 1.0);
    nh_.param("apf_repulsive_gain", repulsive_gain_, 5.0);
    nh_.param("apf_obstacle_influence_distance", obstacle_influence_distance_, 0.3);
    nh_.param("apf_decay_rate", decay_rate_, 0.1);
    nh_.param("apf_max_force", max_force_magnitude_, 0.5);
    
    // 读取排斥力配置参数
    nh_.param("apf_use_repulsive_force", use_repulsive_force_, false);
    nh_.param("apf_use_numerical_gradient", use_numerical_gradient_, true);
    nh_.param("apf_gradient_delta", gradient_delta_, 0.01);
    nh_.param("apf_repulsive_force_threshold", repulsive_force_threshold_, 0.5);
    
    // 验证参数
    if (attractive_gain_ < 0.0)
    {
        ROS_WARN("[%s] attractive_gain (%.3f) < 0, using absolute value", 
                 name_.c_str(), attractive_gain_);
        attractive_gain_ = std::abs(attractive_gain_);
    }
    
    if (repulsive_gain_ < 0.0)
    {
        ROS_WARN("[%s] repulsive_gain (%.3f) < 0, using absolute value", 
                 name_.c_str(), repulsive_gain_);
        repulsive_gain_ = std::abs(repulsive_gain_);
    }
    
    if (obstacle_influence_distance_ <= 0.0)
    {
        ROS_WARN("[%s] obstacle_influence_distance (%.3f) <= 0, using default 0.3", 
                 name_.c_str(), obstacle_influence_distance_);
        obstacle_influence_distance_ = 0.3;
    }
    
    if (max_force_magnitude_ <= 0.0)
    {
        ROS_WARN("[%s] max_force (%.3f) <= 0, using default 0.5", 
                 name_.c_str(), max_force_magnitude_);
        max_force_magnitude_ = 0.5;
    }
    
    if (gradient_delta_ <= 0.0)
    {
        ROS_WARN("[%s] gradient_delta (%.4f) <= 0, using default 0.01", 
                 name_.c_str(), gradient_delta_);
        gradient_delta_ = 0.01;
    }
    
    ROS_INFO("[%s] Initialized:", name_.c_str());
    ROS_INFO("[%s]   Attractive: K_att=%.3f", name_.c_str(), attractive_gain_);
    ROS_INFO("[%s]   Repulsive: η=%.3f, ρ₀=%.3f, σ=%.3f", 
             name_.c_str(), repulsive_gain_, obstacle_influence_distance_, decay_rate_);
    ROS_INFO("[%s]   Repulsive force: %s", name_.c_str(), 
             use_repulsive_force_ ? "ENABLED" : "DISABLED");
    if (use_repulsive_force_)
    {
        ROS_INFO("[%s]   Numerical gradient: %s (delta=%.4f rad)", 
                 name_.c_str(), use_numerical_gradient_ ? "ENABLED" : "DISABLED", gradient_delta_);
        ROS_INFO("[%s]   Threshold: %.3f m", name_.c_str(), repulsive_force_threshold_);
    }
    ROS_INFO("[%s]   Max force: %.3f", name_.c_str(), max_force_magnitude_);
    
    return true;
}

void APFModule::postExtend(PlanningContext& context)
{
    // 检查必要的信息
    if (!context.goal_state || !context.new_node_candidate)
    {
        ROS_WARN_THROTTLE(10.0, "[%s] Missing goal_state or new_node_candidate", name_.c_str());
        return;
    }
    
    // 计算吸引力和排斥力
    Eigen::VectorXd attractive_force = calculateAttractiveForce(*context.new_node_candidate, 
                                                                *context.goal_state);
    Eigen::VectorXd repulsive_force = calculateRepulsiveForce(*context.new_node_candidate, 
                                                              context.obstacle_info);
    
    // 合成总力
    Eigen::VectorXd total_force = attractive_force + repulsive_force;
    
    // 限制力的大小
    double force_magnitude = total_force.norm();
    if (force_magnitude > max_force_magnitude_)
    {
        total_force = total_force * (max_force_magnitude_ / force_magnitude);
        ROS_DEBUG_THROTTLE(5.0, "[%s] Force magnitude limited: %.4f -> %.4f", 
                         name_.c_str(), force_magnitude, max_force_magnitude_);
    }
    
    // 对新节点施加力
    applyForce(*context.new_node_candidate, total_force, context.step_size);
    
    ROS_DEBUG_THROTTLE(10.0, "[%s] Applied force: att_mag=%.4f, rep_mag=%.4f, total_mag=%.4f", 
                     name_.c_str(), attractive_force.norm(), repulsive_force.norm(), 
                     total_force.norm());
}

Eigen::VectorXd APFModule::calculateAttractiveForce(const State& current, 
                                                     const State& goal) const
{
    size_t dim = current.values.size();
    Eigen::VectorXd force = Eigen::VectorXd::Zero(dim);
    
    if (dim != goal.values.size())
    {
        ROS_ERROR_THROTTLE(10.0, "[%s] State dimension mismatch: %zu vs %zu", 
                         name_.c_str(), dim, goal.values.size());
        return force;
    }
    
    // 改进的吸引力（基于论文3）：F_att = K_att * (goal - current)
    // 线性吸引力，简单有效
    for (size_t i = 0; i < dim; ++i)
    {
        force(i) = attractive_gain_ * (goal.values[i] - current.values[i]);
    }
    
    return force;
}

Eigen::VectorXd APFModule::calculateRepulsiveForce(const State& current,
                                                    const ObstacleInfo& obstacles) const
{
    size_t dim = current.values.size();
    Eigen::VectorXd force = Eigen::VectorXd::Zero(dim);
    
    // 如果没有障碍物信息或距离太远，不施加排斥力
    if (!std::isfinite(obstacles.min_obstacle_distance) || 
        obstacles.min_obstacle_distance > obstacle_influence_distance_)
    {
        return force;
    }
    
    double d_obs = obstacles.min_obstacle_distance;
    const double epsilon = 0.01;  // 避免除零
    d_obs = std::max(d_obs, epsilon);
    
    // 基于论文1的指数衰减排斥力
    // F_rep = (η/σ) * (e^((ρ₀-d)/σ) - 1) * e^((ρ₀-d)/σ) * direction
    // 其中：
    // - η = repulsive_gain_
    // - σ = decay_rate_（衰减率，需要添加）
    // - ρ₀ = obstacle_influence_distance_
    // - d = d_obs
    
    if (d_obs <= obstacle_influence_distance_)
    {
        // 基于论文1的指数衰减排斥力
        // F_rep = (η/σ) * (e^((ρ₀-d)/σ) - 1) * e^((ρ₀-d)/σ) * direction
        
        // 计算指数项
        double exponent = (obstacle_influence_distance_ - d_obs) / decay_rate_;
        
        // 防止指数溢出
        if (exponent > 10.0)
        {
            exponent = 10.0;
        }
        
        double exp_term = std::exp(exponent);
        
        // 计算排斥力大小
        // F_rep_magnitude = (η/σ) * (e^x - 1) * e^x
        double rep_magnitude = (repulsive_gain_ / decay_rate_) * (exp_term - 1.0) * exp_term;
        
        // 限制排斥力大小，避免过大
        const double max_rep_magnitude = 100.0;
        if (rep_magnitude > max_rep_magnitude)
        {
            rep_magnitude = max_rep_magnitude;
        }
        
        // 计算方向：远离最近的障碍物
        // 注意：在关节空间中，我们无法直接计算笛卡尔空间的障碍物方向
        // 因此，我们采用保守策略：不施加排斥力，只依赖碰撞检测
        // 
        // 原因：
        // 1. 障碍物点在笛卡尔空间，需要IK才能映射到关节空间
        // 2. 错误的方向可能导致节点被推向障碍物
        // 3. RRT*的碰撞检测已经足够避障
        //
        // 如果未来需要APF排斥力，需要实现：
        // - FK/IK转换
        // - 雅可比矩阵计算方向
        // - 或使用数值梯度
        
        // 计算排斥力方向
        if (use_repulsive_force_)
        {
            if (use_numerical_gradient_)
            {
                // 使用数值梯度计算方向（准确但慢）
                Eigen::VectorXd gradient = calculateNumericalGradient(current, d_obs);
                
                double gradient_norm = gradient.norm();
                if (gradient_norm > 1e-6)
                {
                    // 归一化并缩放
                    force = (rep_magnitude / gradient_norm) * gradient;
                    
                    ROS_DEBUG_THROTTLE(5.0, "[%s] Repulsive force: d_obs=%.4f, magnitude=%.4f, gradient_norm=%.4f", 
                                     name_.c_str(), d_obs, rep_magnitude, gradient_norm);
                }
                else
                {
                    ROS_DEBUG_THROTTLE(5.0, "[%s] Gradient too small, no repulsive force applied", name_.c_str());
                }
            }
            else
            {
                // 简化方法：向关节中间位置推（快速但不准确）
                // 假设关节中间位置（0弧度）通常更安全
                for (size_t i = 0; i < dim; ++i)
                {
                    double direction = (0.0 - current.values[i]) > 0 ? 1.0 : -1.0;
                    force(i) = rep_magnitude * direction / std::sqrt(static_cast<double>(dim));
                }
                
                ROS_DEBUG_THROTTLE(5.0, "[%s] Repulsive force (heuristic): d_obs=%.4f, magnitude=%.4f", 
                                 name_.c_str(), d_obs, rep_magnitude);
            }
        }
        else
        {
            ROS_DEBUG_THROTTLE(5.0, "[%s] Repulsive force disabled (d_obs=%.4f)", 
                             name_.c_str(), d_obs);
        }
    }
    
    return force;
}

Eigen::VectorXd APFModule::calculateNumericalGradient(const State& current, 
                                                       double current_dist) const
{
    size_t dim = current.values.size();
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dim);
    
    // 对每个关节计算数值梯度
    for (size_t i = 0; i < dim; ++i)
    {
        // 正方向扰动
        State perturbed_pos = current;
        perturbed_pos.values[i] += gradient_delta_;
        double dist_pos = getMinObstacleDistance(perturbed_pos);
        
        // 负方向扰动
        State perturbed_neg = current;
        perturbed_neg.values[i] -= gradient_delta_;
        double dist_neg = getMinObstacleDistance(perturbed_neg);
        
        // 中心差分：gradient = (f(x+h) - f(x-h)) / (2h)
        // 正梯度表示增大关节角度会增加距离
        gradient(i) = (dist_pos - dist_neg) / (2.0 * gradient_delta_);
    }
    
    return gradient;
}

double APFModule::getMinObstacleDistance(const State& state) const
{
    // 注意：这个函数需要访问planning_scene来计算距离
    // 但APFModule目前没有planning_scene的引用
    // 
    // 临时解决方案：返回一个大值，表示无法计算
    // 这意味着数值梯度方法目前无法工作
    // 
    // 要启用数值梯度，需要：
    // 1. 修改APFModule::initialize接收planning_scene
    // 2. 在这里使用MoveIt的碰撞检测API
    
    ROS_WARN_ONCE("[%s] getMinObstacleDistance not fully implemented, numerical gradient disabled", 
                  name_.c_str());
    
    return std::numeric_limits<double>::infinity();
}

void APFModule::applyForce(State& state, const Eigen::VectorXd& force, double step_size) const
{
    if (static_cast<size_t>(force.size()) != state.values.size())
    {
        ROS_ERROR_THROTTLE(10.0, "[%s] Force dimension mismatch", name_.c_str());
        return;
    }
    
    // 将力转换为位置调整
    // 使用更小的缩放因子，避免调整过大导致穿过障碍物
    // 从 0.1 减小到 0.02，使调整更温和
    double scale = step_size * 0.02;  // 更保守的缩放因子
    
    for (size_t i = 0; i < state.values.size(); ++i)
    {
        state.values[i] += force(i) * scale;
    }
    
    // 注意：这里没有检查关节限制
    // 在实际应用中，应该确保调整后的状态仍在有效范围内
}

} // namespace modules
} // namespace improved_rrt_planner

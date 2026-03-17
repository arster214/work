#include "improved_rrt_planner/modules/variable_step_module.h"
#include "improved_rrt_planner/improved_rrt.h"
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace improved_rrt_planner
{
namespace modules
{

VariableStepModule::VariableStepModule()
    : min_step_size_(0.1)
    , max_step_size_(1.0)
    , density_weight_(0.5)
    , distance_weight_(0.5)
{
}

bool VariableStepModule::initialize(const ros::NodeHandle& nh, const std::string& name)
{
    nh_ = nh;
    name_ = name;
    
    // 读取参数
    nh_.param("variable_step_min", min_step_size_, 0.1);
    nh_.param("variable_step_max", max_step_size_, 1.0);
    nh_.param("variable_step_density_weight", density_weight_, 0.5);
    nh_.param("variable_step_distance_weight", distance_weight_, 0.5);
    
    // 验证参数
    if (min_step_size_ <= 0.0 || max_step_size_ <= 0.0)
    {
        ROS_ERROR("[%s] Invalid step size: min=%.3f, max=%.3f", 
                 name_.c_str(), min_step_size_, max_step_size_);
        return false;
    }
    
    if (min_step_size_ > max_step_size_)
    {
        ROS_WARN("[%s] min_step_size (%.3f) > max_step_size (%.3f), swapping", 
                 name_.c_str(), min_step_size_, max_step_size_);
        std::swap(min_step_size_, max_step_size_);
    }
    
    if (density_weight_ < 0.0 || density_weight_ > 1.0)
    {
        ROS_WARN("[%s] density_weight (%.3f) out of range [0,1], clamping", 
                 name_.c_str(), density_weight_);
        density_weight_ = std::max(0.0, std::min(1.0, density_weight_));
    }
    
    if (distance_weight_ < 0.0 || distance_weight_ > 1.0)
    {
        ROS_WARN("[%s] distance_weight (%.3f) out of range [0,1], clamping", 
                 name_.c_str(), distance_weight_);
        distance_weight_ = std::max(0.0, std::min(1.0, distance_weight_));
    }
    
    ROS_INFO("[%s] Initialized: min_step=%.3f, max_step=%.3f, density_weight=%.3f, distance_weight=%.3f",
             name_.c_str(), min_step_size_, max_step_size_, density_weight_, distance_weight_);
    
    return true;
}

void VariableStepModule::preExtend(PlanningContext& context)
{
    // 计算步长并更新到 context
    double step_size = calculateStepSize(context.obstacle_info);
    context.step_size = step_size;
    
    ROS_DEBUG_THROTTLE(5.0, "[%s] Adjusted step size: %.4f (density: %.3f, min_dist: %.3f)", 
                     name_.c_str(), step_size, 
                     context.obstacle_info.obstacle_density,
                     context.obstacle_info.min_obstacle_distance);
}

double VariableStepModule::calculateStepSize(const ObstacleInfo& obstacles) const
{
    // 基础步长从最大值开始
    double step = max_step_size_;
    
    // 根据障碍物密度减小步长
    // density 越大，步长越小
    if (density_weight_ > 0.0 && obstacles.obstacle_density > 0.0)
    {
        double density_factor = 1.0 - density_weight_ * std::min(1.0, obstacles.obstacle_density);
        step *= density_factor;
    }
    
    // 根据障碍物距离减小步长
    // 距离越近，步长越小
    if (distance_weight_ > 0.0 && std::isfinite(obstacles.min_obstacle_distance))
    {
        // 使用指数衰减：距离越近，影响越大
        const double epsilon = 0.1;  // 避免除零
        double distance_factor = 1.0 - distance_weight_ / (obstacles.min_obstacle_distance + epsilon);
        distance_factor = std::max(0.0, distance_factor);  // 确保非负
        step *= distance_factor;
    }
    
    // 限制在 [min_step_size, max_step_size] 范围内
    step = std::max(min_step_size_, std::min(max_step_size_, step));
    
    return step;
}

} // namespace modules
} // namespace improved_rrt_planner

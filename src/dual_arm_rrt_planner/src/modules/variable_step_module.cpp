#include "dual_arm_rrt_planner/modules/variable_step_module.h"
#include "dual_arm_rrt_planner/dual_arm_rrt.h"
#include <ros/ros.h>

namespace dual_arm_planner
{
namespace modules
{

VariableStepModule::VariableStepModule()
    : min_step_size_(0.1)
    , max_step_size_(1.0)
    , density_weight_(1.0)
    , distance_weight_(1.0)
{
}

bool VariableStepModule::initialize(const ros::NodeHandle& nh, const std::string& name)
{
    nh_ = nh;
    name_ = name;
    nh_.param("variable_min_step", min_step_size_, 0.1);
    nh_.param("variable_max_step", max_step_size_, 1.0);
    nh_.param("variable_density_weight", density_weight_, 1.0);
    nh_.param("variable_distance_weight", distance_weight_, 1.0);
    ROS_INFO("[%s] VariableStepModule initialized: min=%.3f max=%.3f", name_.c_str(), min_step_size_, max_step_size_);
    return true;
}

void VariableStepModule::preExtend(PlanningContext& context)
{
    double step = calculateStepSize(context.obstacle_info);
    context.step_size = step;
    ROS_DEBUG_THROTTLE(10.0, "[%s] Adjusted step_size to %.4f", name_.c_str(), step);
}

double VariableStepModule::calculateStepSize(const ObstacleInfo& obstacles) const
{
    double d = obstacles.min_obstacle_distance;
    if (!std::isfinite(d)) return (min_step_size_ + max_step_size_) * 0.5;
    double t = std::max(0.0, std::min(1.0, (d / (obstacles.obstacle_density + 1e-6))));
    double step = min_step_size_ + (max_step_size_ - min_step_size_) * t;
    return step;
}

} // namespace modules
} // namespace dual_arm_planner

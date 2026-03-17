#include "dual_arm_rrt_planner/modules/goal_bias_module.h"
#include "dual_arm_rrt_planner/dual_arm_rrt.h"
#include <ros/ros.h>

namespace dual_arm_planner
{
namespace modules
{

GoalBiasModule::GoalBiasModule()
    : goal_bias_(0.05)
    , rng_(std::random_device{}())
    , dist_(0.0, 1.0)
{
}

bool GoalBiasModule::initialize(const ros::NodeHandle& nh, const std::string& name)
{
    nh_ = nh;
    name_ = name;
    nh_.param("goal_bias", goal_bias_, 0.05);
    if (goal_bias_ < 0.0 || goal_bias_ > 1.0) goal_bias_ = std::max(0.0, std::min(1.0, goal_bias_));
    ROS_INFO("[%s] Initialized with goal_bias = %.3f", name_.c_str(), goal_bias_);
    return true;
}

bool GoalBiasModule::preSample(PlanningContext& context)
{
    if (!context.goal_state)
    {
        ROS_WARN_THROTTLE(10.0, "[%s] Goal state is null, cannot apply goal bias", name_.c_str());
        return false;
    }
    if (dist_(rng_) < goal_bias_)
    {
        if (context.current_sample)
        {
            *context.current_sample = *context.goal_state;
            ROS_DEBUG_THROTTLE(5.0, "[%s] Sampling goal state (bias: %.3f)", name_.c_str(), goal_bias_);
            return true;
        }
        else
        {
            ROS_WARN_THROTTLE(10.0, "[%s] current_sample pointer is null", name_.c_str());
            return false;
        }
    }
    return false;
}

} // namespace modules
} // namespace dual_arm_planner

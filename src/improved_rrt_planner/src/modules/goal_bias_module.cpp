#include "improved_rrt_planner/modules/goal_bias_module.h"
#include "improved_rrt_planner/improved_rrt.h"  // 包含 State 的完整定义
#include <ros/ros.h>

namespace improved_rrt_planner
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
    
    // 读取 goal_bias 参数（默认值：0.05，即5%的概率采样目标）
    nh_.param("goal_bias", goal_bias_, 0.05);
    
    // 验证参数范围
    if (goal_bias_ < 0.0 || goal_bias_ > 1.0)
    {
        ROS_WARN("[%s] Invalid goal_bias value: %.3f, clamping to [0.0, 1.0]", 
                 name_.c_str(), goal_bias_);
        goal_bias_ = std::max(0.0, std::min(1.0, goal_bias_));
    }
    
    ROS_INFO("[%s] Initialized with goal_bias = %.3f", name_.c_str(), goal_bias_);
    
    return true;
}

bool GoalBiasModule::preSample(PlanningContext& context)
{
    // 检查目标状态是否有效
    if (!context.goal_state)
    {
        ROS_WARN_THROTTLE(10.0, "[%s] Goal state is null, cannot apply goal bias", 
                         name_.c_str());
        return false;
    }
    
    // 以 goal_bias 概率返回目标状态
    if (dist_(rng_) < goal_bias_)
    {
        // 将目标状态设置为当前采样点
        if (context.current_sample)
        {
            *context.current_sample = *context.goal_state;
            
            ROS_DEBUG_THROTTLE(5.0, "[%s] Sampling goal state (bias: %.3f)", 
                             name_.c_str(), goal_bias_);
            
            return true;  // 告诉规划器我们提供了采样点
        }
        else
        {
            ROS_WARN_THROTTLE(10.0, "[%s] current_sample pointer is null", name_.c_str());
            return false;
        }
    }
    
    // 不采样目标，让其他模块或默认采样器处理
    return false;
}

} // namespace modules
} // namespace improved_rrt_planner

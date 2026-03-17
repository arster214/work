#include "improved_rrt_planner/modules/t_rrt_module.h"
#include "improved_rrt_planner/improved_rrt.h"  // 包含 State 的完整定义
#include <ros/ros.h>
#include <cmath>
#include <limits>

namespace improved_rrt_planner
{
namespace modules
{

TRRTModule::TRRTModule()
    : temp_(100.0)
    , init_temperature_(100.0)
    , temp_change_factor_(std::exp(0.1))
    , worst_cost_(std::numeric_limits<double>::infinity())
    , best_cost_(std::numeric_limits<double>::infinity())
    , cost_threshold_(std::numeric_limits<double>::infinity())
{
}

bool TRRTModule::initialize(const ros::NodeHandle& nh, const std::string& name)
{
    nh_ = nh;
    name_ = name;
    
    // 读取参数
    nh_.param("t_rrt_init_temperature", init_temperature_, 100.0);
    
    double temp_change_factor_param = 0.1;
    nh_.param("t_rrt_temp_change_factor", temp_change_factor_param, 0.1);
    temp_change_factor_ = std::exp(temp_change_factor_param);
    
    nh_.param("t_rrt_cost_threshold", cost_threshold_, 
              std::numeric_limits<double>::infinity());
    
    // 初始化温度
    temp_ = init_temperature_;
    
    // 验证参数
    if (init_temperature_ <= 0.0)
    {
        ROS_WARN("[%s] Invalid init_temperature: %.3f, using default 100.0", 
                 name_.c_str(), init_temperature_);
        init_temperature_ = 100.0;
        temp_ = init_temperature_;
    }
    
    if (temp_change_factor_ <= 0.0)
    {
        ROS_WARN("[%s] Invalid temp_change_factor: %.3f, using default exp(0.1)", 
                 name_.c_str(), temp_change_factor_);
        temp_change_factor_ = std::exp(0.1);
    }
    
    ROS_INFO("[%s] Initialized with init_temperature=%.2f, temp_change_factor=%.4f, cost_threshold=%.2f",
             name_.c_str(), init_temperature_, temp_change_factor_, cost_threshold_);
    
    return true;
}

void TRRTModule::postExtend(PlanningContext& context)
{
    // 执行转换测试
    if (!transitionTest(context.motion_cost))
    {
        // 转换测试失败，拒绝此节点
        context.is_valid = false;
        
        ROS_DEBUG_THROTTLE(5.0, "[%s] Transition test failed for cost %.4f (temp: %.2f)", 
                         name_.c_str(), context.motion_cost, temp_);
    }
    else
    {
        ROS_DEBUG_THROTTLE(10.0, "[%s] Transition test passed for cost %.4f (temp: %.2f)", 
                         name_.c_str(), context.motion_cost, temp_);
    }
    
    // 更新最差和最优代价
    if (context.accumulated_cost > worst_cost_)
    {
        worst_cost_ = context.accumulated_cost;
    }
    
    if (context.accumulated_cost < best_cost_)
    {
        best_cost_ = context.accumulated_cost;
    }
}

void TRRTModule::cleanup()
{
    // 重置温度和代价统计
    temp_ = init_temperature_;
    worst_cost_ = std::numeric_limits<double>::infinity();
    best_cost_ = std::numeric_limits<double>::infinity();
    
    ROS_DEBUG("[%s] Cleanup: temperature reset to %.2f", name_.c_str(), temp_);
}

bool TRRTModule::transitionTest(double motion_cost)
{
    // 安全检查：拒绝无效代价
    if (!std::isfinite(motion_cost) || motion_cost < 0)
    {
        ROS_WARN_THROTTLE(1.0, "[%s] Invalid motion cost: %f", name_.c_str(), motion_cost);
        return false;
    }
    
    // 拒绝超过阈值的代价
    if (motion_cost >= cost_threshold_)
    {
        return false;
    }
    
    // 接受接近零或负代价
    if (motion_cost < 1e-4)
    {
        return true;
    }
    
    // 安全检查：温度必须为正
    if (temp_ <= 0 || !std::isfinite(temp_))
    {
        ROS_ERROR_ONCE("[%s] Invalid temperature: %f, resetting to init_temperature", 
                      name_.c_str(), temp_);
        temp_ = init_temperature_;
    }
    
    // 基于温度的概率接受
    double transition_prob = std::exp(-motion_cost / temp_);
    
    // 安全检查：概率必须有效
    if (!std::isfinite(transition_prob))
    {
        ROS_WARN_THROTTLE(1.0, "[%s] Invalid transition probability, accepting motion", 
                         name_.c_str());
        return true;
    }
    
    if (transition_prob > 0.5)
    {
        // 转换成功，略微降低温度
        double cost_range = worst_cost_ - best_cost_;
        if (std::isfinite(cost_range) && std::abs(cost_range) > 1e-4)
        {
            double temp_factor = std::exp(motion_cost / (0.1 * cost_range));
            if (std::isfinite(temp_factor) && temp_factor > 0)
            {
                temp_ /= temp_factor;
            }
        }
        return true;
    }
    
    // 转换失败，略微提高温度
    if (std::isfinite(temp_change_factor_) && temp_change_factor_ > 0)
    {
        temp_ *= temp_change_factor_;
        // 限制温度上限
        if (temp_ > init_temperature_ * 10.0)
        {
            temp_ = init_temperature_ * 10.0;
        }
    }
    
    return false;
}

} // namespace modules
} // namespace improved_rrt_planner

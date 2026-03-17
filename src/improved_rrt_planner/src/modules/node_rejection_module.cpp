#include "improved_rrt_planner/modules/node_rejection_module.h"
#include "improved_rrt_planner/improved_rrt.h"
#include <ros/ros.h>
#include <cmath>

namespace improved_rrt_planner
{
namespace modules
{

NodeRejectionModule::NodeRejectionModule()
    : rejection_radius_factor_(1.0)
{
}

bool NodeRejectionModule::initialize(const ros::NodeHandle& nh, const std::string& name)
{
    nh_ = nh;
    name_ = name;
    
    // 读取参数
    nh_.param("node_rejection_radius_factor", rejection_radius_factor_, 1.0);
    
    // 验证参数
    if (rejection_radius_factor_ <= 0.0)
    {
        ROS_WARN("[%s] rejection_radius_factor (%.3f) <= 0, using default 1.0", 
                 name_.c_str(), rejection_radius_factor_);
        rejection_radius_factor_ = 1.0;
    }
    
    ROS_INFO("[%s] Initialized with rejection_radius_factor = %.3f",
             name_.c_str(), rejection_radius_factor_);
    
    return true;
}

void NodeRejectionModule::postExtend(PlanningContext& context)
{
    // 检查必要的信息
    if (!context.new_node_candidate || !context.tree)
    {
        ROS_WARN_THROTTLE(10.0, "[%s] Missing new_node_candidate or tree", name_.c_str());
        return;
    }
    
    // 计算拒绝半径
    double rejection_radius = rejection_radius_factor_ * context.step_size;
    
    // 检查是否有节点在拒绝半径内
    if (hasNearbyNodes(*context.new_node_candidate, rejection_radius, *context.tree))
    {
        // 拒绝此节点
        context.is_valid = false;
        
        ROS_DEBUG_THROTTLE(5.0, "[%s] Node rejected: nearby node within radius %.4f", 
                         name_.c_str(), rejection_radius);
    }
}

bool NodeRejectionModule::hasNearbyNodes(const State& candidate, double radius,
                                         const std::vector<Motion>& tree) const
{
    // 优化：使用边界框预过滤
    // 计算候选节点的边界框
    double radius_squared = radius * radius;
    
    // 遍历树中的所有节点
    for (const auto& motion : tree)
    {
        // 计算距离
        double dist = distance(candidate, motion.state);
        
        if (dist < radius)
        {
            return true;  // 找到了附近的节点
        }
    }
    
    return false;  // 没有附近的节点
}

double NodeRejectionModule::distance(const State& a, const State& b) const
{
    if (a.values.size() != b.values.size())
    {
        ROS_ERROR_THROTTLE(10.0, "[%s] State dimension mismatch: %zu vs %zu", 
                         name_.c_str(), a.values.size(), b.values.size());
        return std::numeric_limits<double>::infinity();
    }
    
    double sum = 0.0;
    for (size_t i = 0; i < a.values.size(); ++i)
    {
        double diff = a.values[i] - b.values[i];
        sum += diff * diff;
    }
    
    return std::sqrt(sum);
}

} // namespace modules
} // namespace improved_rrt_planner

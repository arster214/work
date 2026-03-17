#include "dual_arm_rrt_planner/modules/node_rejection_module.h"
#include "dual_arm_rrt_planner/dual_arm_rrt.h"
#include <ros/ros.h>
#include <cmath>

namespace dual_arm_planner
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
    nh_.param("node_rejection_radius_factor", rejection_radius_factor_, 1.0);
    if (rejection_radius_factor_ <= 0.0) rejection_radius_factor_ = 1.0;
    ROS_INFO("[%s] Initialized with rejection_radius_factor = %.3f", name_.c_str(), rejection_radius_factor_);
    return true;
}

void NodeRejectionModule::postExtend(PlanningContext& context)
{
    if (!context.new_node_candidate || !context.tree)
    {
        ROS_WARN_THROTTLE(10.0, "[%s] Missing new_node_candidate or tree", name_.c_str());
        return;
    }
    double rejection_radius = rejection_radius_factor_ * context.step_size;
    if (hasNearbyNodes(*context.new_node_candidate, rejection_radius, *context.tree))
    {
        context.is_valid = false;
        ROS_DEBUG_THROTTLE(5.0, "[%s] Node rejected: nearby node within radius %.4f", name_.c_str(), rejection_radius);
    }
}

bool NodeRejectionModule::hasNearbyNodes(const State& candidate, double radius, const std::vector<Motion*>& tree) const
{
    double radius_squared = radius * radius;
    for (const auto& motion_ptr : tree)
    {
        if (!motion_ptr) continue;
        double dist = distance(candidate, motion_ptr->state);
        if (dist < radius) return true;
    }
    return false;
}

double NodeRejectionModule::distance(const State& a, const State& b) const
{
    if (a.values.size() != b.values.size())
    {
        ROS_ERROR_THROTTLE(10.0, "[%s] State dimension mismatch: %zu vs %zu", name_.c_str(), a.values.size(), b.values.size());
        return std::numeric_limits<double>::infinity();
    }
    double sum = 0.0;
    for (size_t i = 0; i < a.values.size(); ++i) { double diff = a.values[i] - b.values[i]; sum += diff * diff; }
    return std::sqrt(sum);
}

} // namespace modules
} // namespace dual_arm_planner

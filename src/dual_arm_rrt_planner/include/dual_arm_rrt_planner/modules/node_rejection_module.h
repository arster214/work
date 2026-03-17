#ifndef DUAL_ARM_RRT_PLANNER_NODE_REJECTION_MODULE_H
#define DUAL_ARM_RRT_PLANNER_NODE_REJECTION_MODULE_H

#include "dual_arm_rrt_planner/modules/module_base.h"

namespace dual_arm_planner
{
namespace modules
{

class NodeRejectionModule : public ModuleBase
{
public:
    NodeRejectionModule();
    virtual ~NodeRejectionModule() = default;
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    void postExtend(PlanningContext& context) override;
    std::string getName() const override { return "NodeRejectionModule"; }

private:
    bool hasNearbyNodes(const State& candidate, double radius, const std::vector<Motion*>& tree) const;
    double distance(const State& a, const State& b) const;
    double rejection_radius_factor_;
};

} // namespace modules
} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_NODE_REJECTION_MODULE_H

#ifndef DUAL_ARM_RRT_PLANNER_T_RRT_MODULE_H
#define DUAL_ARM_RRT_PLANNER_T_RRT_MODULE_H

#include "dual_arm_rrt_planner/modules/module_base.h"

namespace dual_arm_planner
{
namespace modules
{

class TRRTModule : public ModuleBase
{
public:
    TRRTModule();
    virtual ~TRRTModule() = default;
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    void postExtend(PlanningContext& context) override;
    void cleanup() override;
    std::string getName() const override { return "TRRTModule"; }

private:
    bool transitionTest(double motion_cost);
    double temp_;
    double init_temperature_;
    double temp_change_factor_;
    double worst_cost_;
    double best_cost_;
    double cost_threshold_;
};

} // namespace modules
} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_T_RRT_MODULE_H

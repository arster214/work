#ifndef DUAL_ARM_RRT_PLANNER_VARIABLE_STEP_MODULE_H
#define DUAL_ARM_RRT_PLANNER_VARIABLE_STEP_MODULE_H

#include "dual_arm_rrt_planner/modules/module_base.h"

namespace dual_arm_planner
{
namespace modules
{

class VariableStepModule : public ModuleBase
{
public:
    VariableStepModule();
    virtual ~VariableStepModule() = default;
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    void preExtend(PlanningContext& context) override;
    std::string getName() const override { return "VariableStepModule"; }

private:
    double calculateStepSize(const ObstacleInfo& obstacles) const;
    double min_step_size_;
    double max_step_size_;
    double density_weight_;
    double distance_weight_;
};

} // namespace modules
} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_VARIABLE_STEP_MODULE_H

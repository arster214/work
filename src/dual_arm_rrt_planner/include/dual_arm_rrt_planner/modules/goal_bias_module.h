#ifndef DUAL_ARM_RRT_PLANNER_GOAL_BIAS_MODULE_H
#define DUAL_ARM_RRT_PLANNER_GOAL_BIAS_MODULE_H

#include "dual_arm_rrt_planner/modules/module_base.h"
#include <random>

namespace dual_arm_planner
{
namespace modules
{

class GoalBiasModule : public ModuleBase
{
public:
    GoalBiasModule();
    virtual ~GoalBiasModule() = default;
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    bool preSample(PlanningContext& context) override;
    std::string getName() const override { return "GoalBiasModule"; }

private:
    double goal_bias_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_;
};

} // namespace modules
} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_GOAL_BIAS_MODULE_H

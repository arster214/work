#pragma once

#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <dual_arm_rrt_planner/dual_arm_collision_interface.h>
#include <dual_arm_rrt_planner/planner_config.h>

namespace dual_arm_rrt_planner
{
class DualArmClearanceObjective : public ompl::base::StateCostIntegralObjective
{
public:
  DualArmClearanceObjective(const ompl::base::SpaceInformationPtr& space_information,
                            const DualArmCollisionInterfacePtr& collision_interface, const PlannerConfig& config);

  ompl::base::Cost stateCost(const ompl::base::State* state) const override;

private:
  DualArmCollisionInterfacePtr collision_interface_;
  const PlannerConfig& config_;
};
}  // namespace dual_arm_rrt_planner

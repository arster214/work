#include <dual_arm_rrt_planner/dual_arm_clearance_objective.h>

#include <stdexcept>

namespace dual_arm_rrt_planner
{
DualArmClearanceObjective::DualArmClearanceObjective(const ompl::base::SpaceInformationPtr& space_information,
                                                     const DualArmCollisionInterfacePtr& collision_interface,
                                                     const PlannerConfig& config)
  : ompl::base::StateCostIntegralObjective(space_information, config.cost_objective.use_motion_cost_interpolation)
  , collision_interface_(collision_interface)
  , config_(config)
{
  if (!collision_interface_)
  {
    throw std::runtime_error("DualArmClearanceObjective received a null collision interface.");
  }
}

ompl::base::Cost DualArmClearanceObjective::stateCost(const ompl::base::State* state) const
{
  if (!state)
  {
    return ompl::base::Cost(config_.cost_objective.invalid_state_cost);
  }

  const double clearance = collision_interface_->clearance(state);
  const double denominator = clearance + config_.cost_objective.clearance_epsilon;
  if (denominator <= 0.0)
  {
    return ompl::base::Cost(config_.cost_objective.invalid_state_cost);
  }

  return ompl::base::Cost(1.0 / denominator);
}
}  // namespace dual_arm_rrt_planner

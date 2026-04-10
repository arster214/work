#include <dual_arm_rrt_planner/dual_arm_heatmap_objective.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace dual_arm_rrt_planner
{
DualArmHeatmapObjective::DualArmHeatmapObjective(const ompl::base::SpaceInformationPtr& space_information,
                                                 ClearanceEvaluator clearance_evaluator,
                                                 const PlannerConfig& config)
  : ompl::base::StateCostIntegralObjective(space_information, config.cost_objective.use_motion_cost_interpolation)
  , clearance_evaluator_(std::move(clearance_evaluator))
  , config_(config)
{
  if (!clearance_evaluator_)
  {
    throw std::runtime_error("DualArmHeatmapObjective requires a valid heatmap clearance evaluator.");
  }
}

ompl::base::Cost DualArmHeatmapObjective::stateCost(const ompl::base::State* state) const
{
  double heatmap_clearance = clearance_evaluator_(state);
  if (!std::isfinite(heatmap_clearance))
  {
    heatmap_clearance = config_.heatmap.safety_clearance_high;
  }

  const double penalty = std::max(0.0, config_.heatmap.objective_clearance_threshold - heatmap_clearance);
  return ompl::base::Cost(penalty);
}
}  // namespace dual_arm_rrt_planner

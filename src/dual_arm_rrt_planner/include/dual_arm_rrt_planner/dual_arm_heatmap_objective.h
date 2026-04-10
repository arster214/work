#pragma once

#include <functional>

#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <dual_arm_rrt_planner/planner_config.h>

namespace dual_arm_rrt_planner
{
class DualArmHeatmapObjective : public ompl::base::StateCostIntegralObjective
{
public:
  using ClearanceEvaluator = std::function<double(const ompl::base::State*)>;

  DualArmHeatmapObjective(const ompl::base::SpaceInformationPtr& space_information,
                          ClearanceEvaluator clearance_evaluator, const PlannerConfig& config);

  ompl::base::Cost stateCost(const ompl::base::State* state) const override;

private:
  ClearanceEvaluator clearance_evaluator_;
  const PlannerConfig& config_;
};
}  // namespace dual_arm_rrt_planner

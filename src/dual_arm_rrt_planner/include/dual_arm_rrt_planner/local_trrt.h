/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Ryan Luna */

#pragma once

#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <boost/random/sobol.hpp>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>

namespace dual_arm_rrt_planner
{
class LocalTRRT : public ompl::base::Planner
{
public:
  enum class SamplingStrategy
  {
    UNIFORM,
    SOBOL
  };

  explicit LocalTRRT(const ompl::base::SpaceInformationPtr& si);

  ~LocalTRRT() override;

  void getPlannerData(ompl::base::PlannerData& data) const override;

  ompl::base::PlannerStatus solve(
      const ompl::base::PlannerTerminationCondition& planner_termination_condition) override;

  void clear() override;

  void setGoalBias(double goal_bias)
  {
    goalBias_ = goal_bias;
  }

  double getGoalBias() const
  {
    return goalBias_;
  }

  void setEnableConnect(bool enable_connect)
  {
    enableConnect_ = enable_connect;
  }

  bool getEnableConnect() const
  {
    return enableConnect_;
  }

  void setSamplingStrategy(const std::string& sampling_strategy);

  std::string getSamplingStrategy() const;

  void setGoalTreeSeedState(const ompl::base::State* state);

  void setSobolSplitIndex(unsigned int sobol_split_index)
  {
    sobolSplitIndex_ = sobol_split_index;
  }

  unsigned int getSobolSplitIndex() const
  {
    return sobolSplitIndex_;
  }

  void setSobolCandidatePoolSize(unsigned int sobol_candidate_pool_size)
  {
    sobolCandidatePoolSize_ = sobol_candidate_pool_size;
  }

  unsigned int getSobolCandidatePoolSize() const
  {
    return sobolCandidatePoolSize_;
  }

  void setSobolSortByGoalDistance(bool sobol_sort_by_goal_distance)
  {
    sobolSortByGoalDistance_ = sobol_sort_by_goal_distance;
  }

  bool getSobolSortByGoalDistance() const
  {
    return sobolSortByGoalDistance_;
  }

  void setRange(double distance)
  {
    maxDistance_ = distance;
  }

  double getRange() const
  {
    return maxDistance_;
  }

  void setTempChangeFactor(double factor)
  {
    tempChangeFactor_ = std::exp(factor);
  }

  double getTempChangeFactor() const
  {
    return std::log(tempChangeFactor_);
  }

  void setCostThreshold(double max_cost)
  {
    costThreshold_ = ompl::base::Cost(max_cost);
  }

  double getCostThreshold() const
  {
    return costThreshold_.value();
  }

  void setInitTemperature(double init_temperature)
  {
    initTemperature_ = init_temperature;
    temp_ = initTemperature_;
  }

  double getInitTemperature() const
  {
    return initTemperature_;
  }

  void setFrontierThreshold(double frontier_threshold)
  {
    frontierThreshold_ = frontier_threshold;
  }

  double getFrontierThreshold() const
  {
    return frontierThreshold_;
  }

  void setFrontierNodeRatio(double frontier_node_ratio)
  {
    frontierNodeRatio_ = frontier_node_ratio;
  }

  double getFrontierNodeRatio() const
  {
    return frontierNodeRatio_;
  }

  void setMinParentChildDistance(double min_parent_child_distance)
  {
    minParentChildDistance_ = min_parent_child_distance;
  }

  double getMinParentChildDistance() const
  {
    return minParentChildDistance_;
  }

  void setMinParentChildDistanceScale(double min_parent_child_distance_scale)
  {
    minParentChildDistanceScale_ = min_parent_child_distance_scale;
  }

  double getMinParentChildDistanceScale() const
  {
    return minParentChildDistanceScale_;
  }

  void setMaxSamplingAttemptsPerIteration(unsigned int max_sampling_attempts_per_iteration)
  {
    maxSamplingAttemptsPerIteration_ = max_sampling_attempts_per_iteration;
  }

  unsigned int getMaxSamplingAttemptsPerIteration() const
  {
    return maxSamplingAttemptsPerIteration_;
  }

  void setSafetyEvaluator(const std::function<double(const ompl::base::State*)>& safety_evaluator)
  {
    safetyEvaluator_ = safety_evaluator;
  }

  void setClearanceEvaluator(const std::function<double(const ompl::base::State*)>& clearance_evaluator)
  {
    clearanceEvaluator_ = clearance_evaluator;
  }

  void setAdaptiveRangeMinScale(double adaptive_range_min_scale)
  {
    adaptiveRangeMinScale_ = adaptive_range_min_scale;
  }

  double getAdaptiveRangeMinScale() const
  {
    return adaptiveRangeMinScale_;
  }

  void setDangerSamplingKeepProbability(double danger_sampling_keep_probability)
  {
    dangerSamplingKeepProbability_ = danger_sampling_keep_probability;
  }

  double getDangerSamplingKeepProbability() const
  {
    return dangerSamplingKeepProbability_;
  }

  template <template <typename T> class NN>
  void setNearestNeighbors()
  {
    if ((nearestNeighbors_ && nearestNeighbors_->size() != 0) ||
        (goalNearestNeighbors_ && goalNearestNeighbors_->size() != 0))
    {
      OMPL_WARN("Calling setNearestNeighbors will clear all states.");
    }

    clear();
    nearestNeighbors_ = std::make_shared<NN<Motion*> >();
    goalNearestNeighbors_ = std::make_shared<NN<Motion*> >();
    setup();
  }

  void setup() override;

protected:
  class Motion;

  using TreeData = std::shared_ptr<ompl::NearestNeighbors<Motion*> >;

  struct SobolCandidate
  {
    std::vector<double> state_values;
    double goal_distance{ 0.0 };
  };

  class Motion
  {
  public:
    Motion() = default;

    explicit Motion(const ompl::base::SpaceInformationPtr& si) : state(si->allocState())
    {
    }

    ~Motion() = default;

    ompl::base::State* state{ nullptr };
    Motion* parent{ nullptr };
    ompl::base::Cost cost;
    bool in_goal_tree{ false };
  };

  enum class GrowResult
  {
    FAILED,
    ADVANCED,
    REACHED
  };

  void freeMemory();

  double distanceFunction(const Motion* a, const Motion* b) const
  {
    return si_->distance(a->state, b->state);
  }

  bool transitionTest(const ompl::base::Cost& motion_cost);

  bool minExpansionControl(double rand_motion_distance);

  void updateBestAndWorstCost(const ompl::base::Cost& cost);

  Motion* addMotionToTree(const ompl::base::State* state, const TreeData& tree, Motion* parent, bool in_goal_tree);

  bool initializeGoalTree(const TreeData& goal_tree, ompl::base::Goal* goal,
                          ompl::base::GoalSampleableRegion* goal_sampleable_region);

  void sampleState(ompl::base::State* state, ompl::base::Goal* goal,
                   ompl::base::GoalSampleableRegion* goal_sampleable_region, bool allow_goal_bias = true);

  bool sampleExpansionTarget(Motion* random_motion, ompl::base::State* random_state,
                             ompl::base::State* interpolated_state, ompl::base::Goal* goal,
                             ompl::base::GoalSampleableRegion* goal_sampleable_region, const TreeData& tree,
                             Motion*& nearest_motion, ompl::base::State*& new_state, double& motion_distance,
                             bool allow_goal_bias);

  void initializeSobolSampler();

  void sampleSobolState(ompl::base::State* state, ompl::base::Goal* goal);

  void refillSobolCandidatePool(ompl::base::Goal* goal);

  double computeGoalDistanceForSorting(ompl::base::Goal* goal, const ompl::base::State* state) const;

  void writeSobolCandidateToState(const SobolCandidate& candidate, ompl::base::State* state) const;

  GrowResult growTreeTowardsState(const TreeData& tree, const ompl::base::State* target_state,
                                  ompl::base::State* interpolated_state, Motion*& result, bool in_goal_tree,
                                  bool enforce_min_parent_child_distance);

  void appendMotionPathToRoot(Motion* motion, std::vector<Motion*>& motion_path) const;

  ompl::base::PlannerStatus solveSingleTree(
      const ompl::base::PlannerTerminationCondition& planner_termination_condition);

  ompl::base::PlannerStatus solveConnect(
      const ompl::base::PlannerTerminationCondition& planner_termination_condition);

  double getEffectiveMinParentChildDistance(double reference_max_distance) const
  {
    if (minParentChildDistanceScale_ > 0.0)
    {
      return reference_max_distance * minParentChildDistanceScale_;
    }

    return minParentChildDistance_;
  }

  double computeSafetyScore(const ompl::base::State* state) const;

  double computeAdaptiveRange(const ompl::base::State* state) const;

  bool acceptStateBySafety(const ompl::base::State* state);

  ompl::base::StateSamplerPtr sampler_;
  TreeData nearestNeighbors_;
  TreeData goalNearestNeighbors_;
  bool enableConnect_{ false };
  SamplingStrategy samplingStrategy_{ SamplingStrategy::UNIFORM };
  double goalBias_{ .05 };
  double maxDistance_{ 0.0 };
  ompl::RNG rng_;
  Motion* lastGoalMotion_{ nullptr };
  ompl::base::State* goalTreeSeedState_{ nullptr };
  unsigned int sobolSplitIndex_{ 0U };
  std::vector<std::unique_ptr<boost::random::sobol>> sobolGenerators_;
  std::vector<double> sobolRandomShifts_;
  std::vector<double> sobolLowerBounds_;
  std::vector<double> sobolUpperBounds_;
  std::deque<SobolCandidate> sobolCandidatePool_;
  unsigned int sobolCandidatePoolSize_{ 32U };
  bool sobolSortByGoalDistance_{ false };
  double temp_{ 0.0 };
  ompl::base::Cost bestCost_;
  ompl::base::Cost worstCost_;
  ompl::base::Cost costThreshold_;
  double tempChangeFactor_{ 0.0 };
  double initTemperature_{ 0.0 };
  double nonfrontierCount_{ 0.0 };
  double frontierCount_{ 0.0 };
  double frontierThreshold_{ 0.0 };
  double frontierNodeRatio_{ 0.0 };
  double minParentChildDistance_{ 0.0 };
  double minParentChildDistanceScale_{ 0.0 };
  unsigned int maxSamplingAttemptsPerIteration_{ 1U };
  std::function<double(const ompl::base::State*)> safetyEvaluator_;
  std::function<double(const ompl::base::State*)> clearanceEvaluator_;
  double adaptiveRangeMinScale_{ 1.0 };
  double dangerSamplingKeepProbability_{ 1.0 };
  ompl::base::OptimizationObjectivePtr opt_;
};
}  // namespace dual_arm_rrt_planner

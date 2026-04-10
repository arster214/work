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

#include <dual_arm_rrt_planner/local_trrt.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/tools/config/SelfConfig.h>

namespace dual_arm_rrt_planner
{
namespace
{
double clampScalar(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

LocalTRRT::LocalTRRT(const ompl::base::SpaceInformationPtr& si) : ompl::base::Planner(si, "TRRT")
{
  specs_.approximateSolutions = true;
  specs_.directed = true;

  Planner::declareParam<double>("range", this, &LocalTRRT::setRange, &LocalTRRT::getRange, "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &LocalTRRT::setGoalBias, &LocalTRRT::getGoalBias, "0.:.05:1.");

  frontierThreshold_ = 0.0;
  setTempChangeFactor(0.1);
  costThreshold_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
  initTemperature_ = 100.0;
  frontierNodeRatio_ = 0.1;

  Planner::declareParam<double>("temp_change_factor", this, &LocalTRRT::setTempChangeFactor,
                                &LocalTRRT::getTempChangeFactor, "0.:.1:1.");
  Planner::declareParam<double>("init_temperature", this, &LocalTRRT::setInitTemperature,
                                &LocalTRRT::getInitTemperature);
  Planner::declareParam<double>("frontier_threshold", this, &LocalTRRT::setFrontierThreshold,
                                &LocalTRRT::getFrontierThreshold);
  Planner::declareParam<double>("frontier_node_ratio", this, &LocalTRRT::setFrontierNodeRatio,
                                &LocalTRRT::getFrontierNodeRatio);
  Planner::declareParam<double>("cost_threshold", this, &LocalTRRT::setCostThreshold,
                                &LocalTRRT::getCostThreshold);
  Planner::declareParam<double>("min_parent_child_distance", this, &LocalTRRT::setMinParentChildDistance,
                                &LocalTRRT::getMinParentChildDistance);
  Planner::declareParam<double>("min_parent_child_distance_scale", this,
                                &LocalTRRT::setMinParentChildDistanceScale,
                                &LocalTRRT::getMinParentChildDistanceScale);
  Planner::declareParam<unsigned int>("max_sampling_attempts_per_iteration", this,
                                      &LocalTRRT::setMaxSamplingAttemptsPerIteration,
                                      &LocalTRRT::getMaxSamplingAttemptsPerIteration);
}

LocalTRRT::~LocalTRRT()
{
  freeMemory();
  if (goalTreeSeedState_ != nullptr)
  {
    si_->freeState(goalTreeSeedState_);
    goalTreeSeedState_ = nullptr;
  }
}

void LocalTRRT::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();

  if (nearestNeighbors_)
  {
    nearestNeighbors_->clear();
  }

  if (goalNearestNeighbors_)
  {
    goalNearestNeighbors_->clear();
  }

  if (goalTreeSeedState_ != nullptr)
  {
    si_->freeState(goalTreeSeedState_);
    goalTreeSeedState_ = nullptr;
  }

  sobolGenerators_.clear();
  sobolRandomShifts_.clear();
  sobolLowerBounds_.clear();
  sobolUpperBounds_.clear();
  sobolCandidatePool_.clear();
  lastGoalMotion_ = nullptr;

  temp_ = initTemperature_;
  nonfrontierCount_ = 1.0;
  frontierCount_ = 1.0;
  if (opt_)
  {
    bestCost_ = worstCost_ = opt_->identityCost();
  }
}

void LocalTRRT::setup()
{
  Planner::setup();

  ompl::tools::SelfConfig self_config(si_, getName());

  if (!pdef_ || !pdef_->hasOptimizationObjective())
  {
    OMPL_INFORM("%s: No optimization objective specified. Defaulting to mechanical work minimization.",
                getName().c_str());
    opt_ = std::make_shared<ompl::base::MechanicalWorkOptimizationObjective>(si_);
  }
  else
  {
    opt_ = pdef_->getOptimizationObjective();
  }

  if (maxDistance_ < std::numeric_limits<double>::epsilon())
  {
    self_config.configurePlannerRange(maxDistance_);
    maxDistance_ *= ompl::magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
  }

  if (frontierThreshold_ < std::numeric_limits<double>::epsilon())
  {
    frontierThreshold_ = si_->getMaximumExtent() * 0.01;
    OMPL_DEBUG("%s: Frontier threshold detected to be %lf", getName().c_str(), frontierThreshold_);
  }

  if (!nearestNeighbors_)
  {
    nearestNeighbors_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
  }

  if (!goalNearestNeighbors_)
  {
    goalNearestNeighbors_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
  }

  nearestNeighbors_->setDistanceFunction([this](const Motion* a, const Motion* b) { return distanceFunction(a, b); });
  goalNearestNeighbors_->setDistanceFunction(
      [this](const Motion* a, const Motion* b) { return distanceFunction(a, b); });

  if (samplingStrategy_ == SamplingStrategy::SOBOL)
  {
    initializeSobolSampler();
  }
  else
  {
    sobolGenerators_.clear();
    sobolRandomShifts_.clear();
    sobolLowerBounds_.clear();
    sobolUpperBounds_.clear();
    sobolCandidatePool_.clear();
  }

  temp_ = initTemperature_;
  nonfrontierCount_ = 1.0;
  frontierCount_ = 1.0;
  bestCost_ = worstCost_ = opt_->identityCost();
}

void LocalTRRT::setSamplingStrategy(const std::string& sampling_strategy)
{
  if (sampling_strategy == "uniform")
  {
    samplingStrategy_ = SamplingStrategy::UNIFORM;
  }
  else if (sampling_strategy == "sobol")
  {
    samplingStrategy_ = SamplingStrategy::SOBOL;
  }
  else
  {
    throw std::runtime_error("Unsupported sampling strategy: " + sampling_strategy);
  }
}

std::string LocalTRRT::getSamplingStrategy() const
{
  return samplingStrategy_ == SamplingStrategy::SOBOL ? "sobol" : "uniform";
}

void LocalTRRT::setGoalTreeSeedState(const ompl::base::State* state)
{
  if (goalTreeSeedState_ != nullptr)
  {
    si_->freeState(goalTreeSeedState_);
    goalTreeSeedState_ = nullptr;
  }

  if (state == nullptr)
  {
    return;
  }

  goalTreeSeedState_ = si_->allocState();
  si_->copyState(goalTreeSeedState_, state);
}

void LocalTRRT::freeMemory()
{
  std::set<Motion*> unique_motions;

  if (nearestNeighbors_)
  {
    std::vector<Motion*> motions;
    nearestNeighbors_->list(motions);
    unique_motions.insert(motions.begin(), motions.end());
  }

  if (goalNearestNeighbors_)
  {
    std::vector<Motion*> motions;
    goalNearestNeighbors_->list(motions);
    unique_motions.insert(motions.begin(), motions.end());
  }

  for (Motion* motion : unique_motions)
  {
    if (motion->state != nullptr)
    {
      si_->freeState(motion->state);
      motion->state = nullptr;
    }
    delete motion;
  }
}

void LocalTRRT::initializeSobolSampler()
{
  const auto* real_vector_state_space =
      dynamic_cast<const ompl::base::RealVectorStateSpace*>(si_->getStateSpace().get());
  if (real_vector_state_space == nullptr)
  {
    throw std::runtime_error("LocalTRRT Sobol sampling requires ompl::base::RealVectorStateSpace.");
  }

  const auto& bounds = real_vector_state_space->getBounds();
  sobolLowerBounds_ = bounds.low;
  sobolUpperBounds_ = bounds.high;
  sobolCandidatePool_.clear();
  sobolGenerators_.clear();

  const std::size_t dimension = real_vector_state_space->getDimension();
  if (dimension == 0U)
  {
    throw std::runtime_error("LocalTRRT Sobol sampling requires a non-empty state dimension.");
  }

  if (sobolSplitIndex_ > 0U && sobolSplitIndex_ < dimension)
  {
    sobolGenerators_.push_back(std::make_unique<boost::random::sobol>(sobolSplitIndex_));
    sobolGenerators_.push_back(std::make_unique<boost::random::sobol>(dimension - sobolSplitIndex_));
  }
  else
  {
    sobolGenerators_.push_back(std::make_unique<boost::random::sobol>(dimension));
  }

  sobolRandomShifts_.resize(dimension);
  for (std::size_t index = 0; index < dimension; ++index)
  {
    sobolRandomShifts_[index] = rng_.uniform01();
  }
}

void LocalTRRT::sampleState(ompl::base::State* state, ompl::base::Goal* goal,
                            ompl::base::GoalSampleableRegion* goal_sampleable_region, bool allow_goal_bias)
{
  if (allow_goal_bias && goal_sampleable_region != nullptr && rng_.uniform01() < goalBias_ &&
      goal_sampleable_region->canSample())
  {
    goal_sampleable_region->sampleGoal(state);
    return;
  }

  if (samplingStrategy_ == SamplingStrategy::SOBOL)
  {
    sampleSobolState(state, allow_goal_bias ? goal : nullptr);
    return;
  }

  sampler_->sampleUniform(state);
}

void LocalTRRT::sampleSobolState(ompl::base::State* state, ompl::base::Goal* goal)
{
  if (sobolGenerators_.empty())
  {
    initializeSobolSampler();
  }

  if (sobolCandidatePool_.empty())
  {
    refillSobolCandidatePool(goal);
  }

  if (sobolCandidatePool_.empty())
  {
    sampler_->sampleUniform(state);
    return;
  }

  writeSobolCandidateToState(sobolCandidatePool_.front(), state);
  sobolCandidatePool_.pop_front();
}

void LocalTRRT::refillSobolCandidatePool(ompl::base::Goal* goal)
{
  if (sobolGenerators_.empty())
  {
    initializeSobolSampler();
  }

  const std::size_t dimension = sobolLowerBounds_.size();
  const double sobol_denominator = static_cast<double>((boost::random::sobol::max)()) + 1.0;
  const std::size_t split_index =
      sobolGenerators_.size() == 2U ? std::min<std::size_t>(sobolSplitIndex_, dimension) : dimension;

  std::vector<SobolCandidate> candidates;
  candidates.reserve(sobolCandidatePoolSize_);

  ompl::base::State* scratch_state = si_->allocState();

  try
  {
    for (unsigned int candidate_index = 0U; candidate_index < sobolCandidatePoolSize_; ++candidate_index)
    {
      SobolCandidate candidate;
      candidate.state_values.resize(dimension);

      for (std::size_t dimension_index = 0U; dimension_index < dimension; ++dimension_index)
      {
        double unit_value = 0.0;
        if (sobolGenerators_.size() == 1U)
        {
          unit_value = static_cast<double>((*sobolGenerators_.front())()) / sobol_denominator;
        }
        else if (dimension_index < split_index)
        {
          unit_value = static_cast<double>((*sobolGenerators_[0])()) / sobol_denominator;
        }
        else
        {
          unit_value = static_cast<double>((*sobolGenerators_[1])()) / sobol_denominator;
        }

        unit_value = std::fmod(unit_value + sobolRandomShifts_[dimension_index], 1.0);

        const double lower_bound = sobolLowerBounds_[dimension_index];
        const double upper_bound = sobolUpperBounds_[dimension_index];
        candidate.state_values[dimension_index] = lower_bound + unit_value * (upper_bound - lower_bound);
      }

      if (sobolSortByGoalDistance_ && goal != nullptr)
      {
        writeSobolCandidateToState(candidate, scratch_state);
        candidate.goal_distance = computeGoalDistanceForSorting(goal, scratch_state);
      }

      candidates.push_back(std::move(candidate));
    }
  }
  catch (const std::range_error&)
  {
    OMPL_WARN("%s: Sobol sequence exhausted. Restarting the sequence from the beginning.", getName().c_str());
    for (const auto& sobol_generator : sobolGenerators_)
    {
      if (sobol_generator)
      {
        sobol_generator->seed();
      }
    }
    si_->freeState(scratch_state);
    refillSobolCandidatePool(goal);
    return;
  }

  si_->freeState(scratch_state);

  if (sobolSortByGoalDistance_ && goal != nullptr)
  {
    std::sort(candidates.begin(), candidates.end(),
              [](const SobolCandidate& left, const SobolCandidate& right)
              { return left.goal_distance < right.goal_distance; });
  }

  sobolCandidatePool_.clear();
  for (const SobolCandidate& candidate : candidates)
  {
    sobolCandidatePool_.push_back(candidate);
  }
}

double LocalTRRT::computeGoalDistanceForSorting(ompl::base::Goal* goal, const ompl::base::State* state) const
{
  if (goal == nullptr)
  {
    return std::numeric_limits<double>::infinity();
  }

  if (auto* goal_region = dynamic_cast<ompl::base::GoalRegion*>(goal))
  {
    return goal_region->distanceGoal(state);
  }

  double distance_to_goal = std::numeric_limits<double>::infinity();
  goal->isSatisfied(state, &distance_to_goal);
  return distance_to_goal;
}

void LocalTRRT::writeSobolCandidateToState(const SobolCandidate& candidate, ompl::base::State* state) const
{
  auto* real_vector_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
  for (std::size_t index = 0U; index < candidate.state_values.size(); ++index)
  {
    real_vector_state->values[index] = candidate.state_values[index];
  }
}

double LocalTRRT::computeSafetyScore(const ompl::base::State* state) const
{
  if (!safetyEvaluator_ || state == nullptr)
  {
    return 1.0;
  }

  const double raw_score = safetyEvaluator_(state);
  if (!std::isfinite(raw_score))
  {
    return 1.0;
  }

  return clampScalar(raw_score, 0.0, 1.0);
}

double LocalTRRT::computeAdaptiveRange(const ompl::base::State* state) const
{
  if (maxDistance_ <= 0.0)
  {
    return maxDistance_;
  }

  const double safety_score = computeSafetyScore(state);
  const double scale = adaptiveRangeMinScale_ + (1.0 - adaptiveRangeMinScale_) * safety_score;
  return std::max(std::numeric_limits<double>::epsilon(), maxDistance_ * scale);
}

bool LocalTRRT::acceptStateBySafety(const ompl::base::State* state)
{
  if (!safetyEvaluator_ || state == nullptr)
  {
    return true;
  }

  const double safety_score = computeSafetyScore(state);
  const double keep_probability =
      dangerSamplingKeepProbability_ + (1.0 - dangerSamplingKeepProbability_) * safety_score;
  return rng_.uniform01() <= clampScalar(keep_probability, 0.0, 1.0);
}

void LocalTRRT::updateBestAndWorstCost(const ompl::base::Cost& cost)
{
  if (opt_->isCostBetterThan(cost, bestCost_))
  {
    bestCost_ = cost;
  }

  if (opt_->isCostBetterThan(worstCost_, cost))
  {
    worstCost_ = cost;
  }
}

LocalTRRT::Motion* LocalTRRT::addMotionToTree(const ompl::base::State* state, const TreeData& tree, Motion* parent,
                                              bool in_goal_tree)
{
  auto* motion = new Motion(si_);
  si_->copyState(motion->state, state);
  motion->parent = parent;
  motion->cost = opt_->stateCost(motion->state);
  motion->in_goal_tree = in_goal_tree;

  tree->add(motion);

  const std::size_t start_tree_size = nearestNeighbors_ ? nearestNeighbors_->size() : 0U;
  const std::size_t goal_tree_size = goalNearestNeighbors_ ? goalNearestNeighbors_->size() : 0U;
  if (start_tree_size + goal_tree_size == 1U)
  {
    bestCost_ = motion->cost;
    worstCost_ = motion->cost;
  }
  else
  {
    updateBestAndWorstCost(motion->cost);
  }

  return motion;
}

bool LocalTRRT::initializeGoalTree(const TreeData& goal_tree, ompl::base::Goal* goal,
                                   ompl::base::GoalSampleableRegion* goal_sampleable_region)
{
  if (!goal_tree || goal_tree->size() != 0U)
  {
    return true;
  }

  ompl::base::State* root_state = nullptr;
  if (goalTreeSeedState_ != nullptr)
  {
    root_state = goalTreeSeedState_;
  }
  else if (goal_sampleable_region != nullptr && goal_sampleable_region->canSample())
  {
    root_state = si_->allocState();
    goal_sampleable_region->sampleGoal(root_state);
  }
  else
  {
    OMPL_ERROR("%s: Connect mode requires a goal state seed or a sampleable goal region.", getName().c_str());
    return false;
  }

  Motion* goal_root = addMotionToTree(root_state, goal_tree, nullptr, true);
  lastGoalMotion_ = goal_root;

  if (root_state != goalTreeSeedState_)
  {
    si_->freeState(root_state);
  }

  if (goal != nullptr)
  {
    double distance_to_goal = std::numeric_limits<double>::infinity();
    if (!goal->isSatisfied(goal_root->state, &distance_to_goal))
    {
      OMPL_WARN("%s: Goal-tree root does not satisfy the requested goal region. Distance to goal = %lf",
                getName().c_str(), distance_to_goal);
    }
  }

  return true;
}

bool LocalTRRT::sampleExpansionTarget(Motion* random_motion, ompl::base::State* random_state,
                                      ompl::base::State* interpolated_state, ompl::base::Goal* goal,
                                      ompl::base::GoalSampleableRegion* goal_sampleable_region, const TreeData& tree,
                                      Motion*& nearest_motion, ompl::base::State*& new_state, double& motion_distance,
                                      bool allow_goal_bias)
{
  if (!tree || tree->size() == 0U)
  {
    return false;
  }

  const unsigned int sampling_attempt_limit =
      maxSamplingAttemptsPerIteration_ > 0U ? maxSamplingAttemptsPerIteration_ : 1U;

  for (unsigned int sampling_attempt = 0U; sampling_attempt < sampling_attempt_limit; ++sampling_attempt)
  {
    sampleState(random_state, goal, goal_sampleable_region, allow_goal_bias);
    nearest_motion = tree->nearest(random_motion);
    if (nearest_motion == nullptr)
    {
      continue;
    }

    const double effective_max_distance = computeAdaptiveRange(nearest_motion->state);
    const double effective_min_parent_child_distance = getEffectiveMinParentChildDistance(effective_max_distance);
    motion_distance = si_->distance(nearest_motion->state, random_state);
    if (motion_distance > effective_max_distance)
    {
      si_->getStateSpace()->interpolate(nearest_motion->state, random_state,
                                        effective_max_distance / motion_distance,
                                        interpolated_state);
      motion_distance = si_->distance(nearest_motion->state, interpolated_state);
      new_state = interpolated_state;
    }
    else
    {
      new_state = random_state;
    }

    if (motion_distance < effective_min_parent_child_distance)
    {
      continue;
    }

    if (!acceptStateBySafety(new_state))
    {
      continue;
    }

    return true;
  }

  return false;
}

LocalTRRT::GrowResult LocalTRRT::growTreeTowardsState(const TreeData& tree, const ompl::base::State* target_state,
                                                      ompl::base::State* interpolated_state, Motion*& result,
                                                      bool in_goal_tree,
                                                      bool enforce_min_parent_child_distance)
{
  result = nullptr;

  if (!tree || tree->size() == 0U || target_state == nullptr)
  {
    return GrowResult::FAILED;
  }

  Motion query_motion;
  query_motion.state = const_cast<ompl::base::State*>(target_state);
  Motion* nearest_motion = tree->nearest(&query_motion);
  if (nearest_motion == nullptr)
  {
    return GrowResult::FAILED;
  }

  double motion_distance = si_->distance(nearest_motion->state, target_state);
  if (motion_distance < std::numeric_limits<double>::epsilon())
  {
    result = nearest_motion;
    return GrowResult::REACHED;
  }

  const ompl::base::State* new_state = target_state;
  bool reached = true;
  const double effective_max_distance = computeAdaptiveRange(nearest_motion->state);
  if (motion_distance > effective_max_distance)
  {
    si_->getStateSpace()->interpolate(nearest_motion->state, target_state, effective_max_distance / motion_distance,
                                      interpolated_state);
    new_state = interpolated_state;
    motion_distance = si_->distance(nearest_motion->state, new_state);
    reached = false;
  }

  if (enforce_min_parent_child_distance &&
      motion_distance < getEffectiveMinParentChildDistance(effective_max_distance))
  {
    return GrowResult::FAILED;
  }

  if (!acceptStateBySafety(new_state))
  {
    return GrowResult::FAILED;
  }

  if (!si_->checkMotion(nearest_motion->state, new_state))
  {
    return GrowResult::FAILED;
  }

  if (!minExpansionControl(motion_distance))
  {
    return GrowResult::FAILED;
  }

  if (!transitionTest(opt_->motionCost(nearest_motion->state, new_state)))
  {
    return GrowResult::FAILED;
  }

  result = addMotionToTree(new_state, tree, nearest_motion, in_goal_tree);
  return reached ? GrowResult::REACHED : GrowResult::ADVANCED;
}

void LocalTRRT::appendMotionPathToRoot(Motion* motion, std::vector<Motion*>& motion_path) const
{
  while (motion != nullptr)
  {
    motion_path.push_back(motion);
    motion = motion->parent;
  }
}

ompl::base::PlannerStatus LocalTRRT::solve(
    const ompl::base::PlannerTerminationCondition& planner_termination_condition)
{
  return enableConnect_ ? solveConnect(planner_termination_condition)
                        : solveSingleTree(planner_termination_condition);
}

ompl::base::PlannerStatus LocalTRRT::solveSingleTree(
    const ompl::base::PlannerTerminationCondition& planner_termination_condition)
{
  checkValidity();

  ompl::base::Goal* goal = pdef_->getGoal().get();
  auto* goal_region = dynamic_cast<ompl::base::GoalSampleableRegion*>(goal);

  while (const ompl::base::State* state = pis_.nextStart())
  {
    addMotionToTree(state, nearestNeighbors_, nullptr, false);
  }

  if (!nearestNeighbors_ || nearestNeighbors_->size() == 0U)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (!sampler_)
  {
    sampler_ = si_->allocStateSampler();
  }

  OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
              nearestNeighbors_->size());
  OMPL_INFORM("%s: Sampling strategy = %s", getName().c_str(), getSamplingStrategy().c_str());
  if (samplingStrategy_ == SamplingStrategy::SOBOL)
  {
    OMPL_INFORM("%s: Sobol candidate pool size = %u, goal-distance sorting = %s, split index = %u",
                getName().c_str(), sobolCandidatePoolSize_, sobolSortByGoalDistance_ ? "true" : "false",
                sobolSplitIndex_);
  }

  Motion* solution = nullptr;
  Motion* approximate_solution = nullptr;
  double approximate_difference = std::numeric_limits<double>::infinity();
  double random_motion_distance = 0.0;

  auto* random_motion = new Motion(si_);
  Motion* nearest_motion = nullptr;
  ompl::base::State* random_state = random_motion->state;
  ompl::base::State* interpolated_state = si_->allocState();
  ompl::base::State* new_state = nullptr;

  while (!planner_termination_condition())
  {
    if (!sampleExpansionTarget(random_motion, random_state, interpolated_state, goal, goal_region,
                               nearestNeighbors_,
                               nearest_motion, new_state, random_motion_distance, true))
    {
      continue;
    }

    if (!si_->checkMotion(nearest_motion->state, new_state))
    {
      continue;
    }

    if (!minExpansionControl(random_motion_distance))
    {
      continue;
    }

    if (!transitionTest(opt_->motionCost(nearest_motion->state, new_state)))
    {
      continue;
    }

    Motion* motion = addMotionToTree(new_state, nearestNeighbors_, nearest_motion, false);

    double distance_to_goal = 0.0;
    const bool is_satisfied = goal->isSatisfied(motion->state, &distance_to_goal);
    if (is_satisfied)
    {
      approximate_difference = distance_to_goal;
      solution = motion;
      break;
    }

    if (distance_to_goal < approximate_difference)
    {
      approximate_difference = distance_to_goal;
      approximate_solution = motion;
    }
  }

  bool solved = false;
  bool approximate = false;

  if (solution == nullptr)
  {
    solution = approximate_solution;
    approximate = true;
  }

  if (solution != nullptr)
  {
    lastGoalMotion_ = solution;

    std::vector<Motion*> motion_path;
    appendMotionPathToRoot(solution, motion_path);

    auto path = std::make_shared<ompl::geometric::PathGeometric>(si_);
    for (int index = static_cast<int>(motion_path.size()) - 1; index >= 0; --index)
    {
      path->append(motion_path[index]->state);
    }

    pdef_->addSolutionPath(path, approximate, approximate_difference, getName());
    solved = true;
  }

  si_->freeState(interpolated_state);
  if (random_motion->state != nullptr)
  {
    si_->freeState(random_motion->state);
    random_motion->state = nullptr;
  }
  delete random_motion;

  OMPL_INFORM("%s: Created %u states", getName().c_str(), nearestNeighbors_->size());

  return { solved, approximate };
}

ompl::base::PlannerStatus LocalTRRT::solveConnect(
    const ompl::base::PlannerTerminationCondition& planner_termination_condition)
{
  checkValidity();

  ompl::base::Goal* goal = pdef_->getGoal().get();
  auto* goal_region = dynamic_cast<ompl::base::GoalSampleableRegion*>(goal);

  while (const ompl::base::State* state = pis_.nextStart())
  {
    addMotionToTree(state, nearestNeighbors_, nullptr, false);
  }

  if (!nearestNeighbors_ || nearestNeighbors_->size() == 0U)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (!initializeGoalTree(goalNearestNeighbors_, goal, goal_region))
  {
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  if (!sampler_)
  {
    sampler_ = si_->allocStateSampler();
  }

  OMPL_INFORM("%s: Starting connect planning with %u start-tree states and %u goal-tree states",
              getName().c_str(), nearestNeighbors_->size(), goalNearestNeighbors_->size());
  OMPL_INFORM("%s: Sampling strategy = %s", getName().c_str(), getSamplingStrategy().c_str());
  if (samplingStrategy_ == SamplingStrategy::SOBOL)
  {
    OMPL_INFORM("%s: Sobol candidate pool size = %u, goal-distance sorting = %s, split index = %u",
                getName().c_str(), sobolCandidatePoolSize_, sobolSortByGoalDistance_ ? "true" : "false",
                sobolSplitIndex_);
  }

  Motion* exact_start_solution = nullptr;
  Motion* exact_goal_solution = nullptr;
  Motion* approximate_solution = nullptr;
  double approximate_difference = std::numeric_limits<double>::infinity();
  bool grow_start_tree = true;

  auto* random_motion = new Motion(si_);
  Motion* nearest_motion = nullptr;
  ompl::base::State* random_state = random_motion->state;
  ompl::base::State* interpolated_state = si_->allocState();
  ompl::base::State* new_state = nullptr;
  double random_motion_distance = 0.0;

  while (!planner_termination_condition())
  {
    const TreeData& active_tree = grow_start_tree ? nearestNeighbors_ : goalNearestNeighbors_;
    const TreeData& other_tree = grow_start_tree ? goalNearestNeighbors_ : nearestNeighbors_;
    const bool active_tree_is_goal_tree = !grow_start_tree;
    const bool other_tree_is_goal_tree = !grow_start_tree ? false : true;
    const bool allow_goal_bias = grow_start_tree;

    if (!sampleExpansionTarget(random_motion, random_state, interpolated_state, goal, goal_region,
                               active_tree,
                               nearest_motion, new_state, random_motion_distance, allow_goal_bias))
    {
      grow_start_tree = !grow_start_tree;
      continue;
    }

    if (!si_->checkMotion(nearest_motion->state, new_state))
    {
      grow_start_tree = !grow_start_tree;
      continue;
    }

    if (!minExpansionControl(random_motion_distance))
    {
      grow_start_tree = !grow_start_tree;
      continue;
    }

    if (!transitionTest(opt_->motionCost(nearest_motion->state, new_state)))
    {
      grow_start_tree = !grow_start_tree;
      continue;
    }

    Motion* added_motion = addMotionToTree(new_state, active_tree, nearest_motion, active_tree_is_goal_tree);

    if (!active_tree_is_goal_tree)
    {
      double distance_to_goal = 0.0;
      const bool is_satisfied = goal->isSatisfied(added_motion->state, &distance_to_goal);
      if (is_satisfied)
      {
        exact_start_solution = added_motion;
        exact_goal_solution = nullptr;
        lastGoalMotion_ = added_motion;
        break;
      }

      if (distance_to_goal < approximate_difference)
      {
        approximate_difference = distance_to_goal;
        approximate_solution = added_motion;
      }
    }

    Motion* connect_motion = nullptr;
    GrowResult connect_result = growTreeTowardsState(other_tree, added_motion->state, interpolated_state,
                                                     connect_motion, other_tree_is_goal_tree, false);
    while (connect_result == GrowResult::ADVANCED && !planner_termination_condition())
    {
      connect_result = growTreeTowardsState(other_tree, added_motion->state, interpolated_state, connect_motion,
                                            other_tree_is_goal_tree, false);
    }

    if (connect_result == GrowResult::REACHED && connect_motion != nullptr)
    {
      if (active_tree_is_goal_tree)
      {
        exact_start_solution = connect_motion;
        exact_goal_solution = added_motion;
      }
      else
      {
        exact_start_solution = added_motion;
        exact_goal_solution = connect_motion;
      }

      lastGoalMotion_ = exact_goal_solution != nullptr ? exact_goal_solution : exact_start_solution;
      break;
    }

    grow_start_tree = !grow_start_tree;
  }

  bool solved = false;
  bool approximate = false;

  if (exact_start_solution != nullptr)
  {
    auto path = std::make_shared<ompl::geometric::PathGeometric>(si_);

    std::vector<Motion*> start_path;
    appendMotionPathToRoot(exact_start_solution, start_path);
    for (int index = static_cast<int>(start_path.size()) - 1; index >= 0; --index)
    {
      path->append(start_path[index]->state);
    }

    if (exact_goal_solution != nullptr)
    {
      std::vector<Motion*> goal_path;
      appendMotionPathToRoot(exact_goal_solution, goal_path);
      for (std::size_t index = 1U; index < goal_path.size(); ++index)
      {
        path->append(goal_path[index]->state);
      }
    }

    pdef_->addSolutionPath(path, false, 0.0, getName());
    solved = true;
  }
  else if (approximate_solution != nullptr)
  {
    std::vector<Motion*> approximate_path;
    appendMotionPathToRoot(approximate_solution, approximate_path);

    auto path = std::make_shared<ompl::geometric::PathGeometric>(si_);
    for (int index = static_cast<int>(approximate_path.size()) - 1; index >= 0; --index)
    {
      path->append(approximate_path[index]->state);
    }

    lastGoalMotion_ = approximate_solution;
    pdef_->addSolutionPath(path, true, approximate_difference, getName());
    solved = true;
    approximate = true;
  }

  si_->freeState(interpolated_state);
  if (random_motion->state != nullptr)
  {
    si_->freeState(random_motion->state);
    random_motion->state = nullptr;
  }
  delete random_motion;

  const unsigned int start_tree_size = nearestNeighbors_ ? nearestNeighbors_->size() : 0U;
  const unsigned int goal_tree_size = goalNearestNeighbors_ ? goalNearestNeighbors_->size() : 0U;
  OMPL_INFORM("%s: Created %u start-tree states and %u goal-tree states", getName().c_str(), start_tree_size,
              goal_tree_size);

  return { solved, approximate };
}

void LocalTRRT::getPlannerData(ompl::base::PlannerData& data) const
{
  Planner::getPlannerData(data);

  if (lastGoalMotion_ != nullptr && !lastGoalMotion_->in_goal_tree)
  {
    data.addGoalVertex(ompl::base::PlannerDataVertex(lastGoalMotion_->state));
  }

  const auto append_tree = [&data](const TreeData& tree) {
    if (!tree)
    {
      return;
    }

    std::vector<Motion*> motions;
    tree->list(motions);

    for (Motion* motion : motions)
    {
      if (motion == nullptr)
      {
        continue;
      }

      if (motion->parent == nullptr)
      {
        if (motion->in_goal_tree)
        {
          data.addGoalVertex(ompl::base::PlannerDataVertex(motion->state));
        }
        else
        {
          data.addStartVertex(ompl::base::PlannerDataVertex(motion->state));
        }
      }
      else
      {
        data.addEdge(ompl::base::PlannerDataVertex(motion->parent->state),
                     ompl::base::PlannerDataVertex(motion->state));
      }
    }
  };

  append_tree(nearestNeighbors_);
  append_tree(goalNearestNeighbors_);
}

bool LocalTRRT::transitionTest(const ompl::base::Cost& motion_cost)
{
  if (!opt_->isCostBetterThan(motion_cost, costThreshold_))
  {
    return false;
  }

  if (motion_cost.value() < 1e-4)
  {
    return true;
  }

  const double delta_cost = motion_cost.value();
  const double transition_probability = std::exp(-delta_cost / temp_);
  if (transition_probability > 0.5)
  {
    const double cost_range = worstCost_.value() - bestCost_.value();
    if (std::fabs(cost_range) > 1e-4)
    {
      temp_ /= std::exp(delta_cost / (0.1 * cost_range));
    }
    return true;
  }

  temp_ *= tempChangeFactor_;
  return false;
}

bool LocalTRRT::minExpansionControl(double random_motion_distance)
{
  if (random_motion_distance > frontierThreshold_)
  {
    ++frontierCount_;
    return true;
  }

  if (static_cast<double>(nonfrontierCount_) / static_cast<double>(frontierCount_) > frontierNodeRatio_)
  {
    return false;
  }

  ++nonfrontierCount_;
  return true;
}
}  // namespace dual_arm_rrt_planner

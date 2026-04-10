#include <dual_arm_rrt_planner/dual_arm_trrt_planner_manager.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/util/RandomNumbers.h>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/String.h>

#include <dual_arm_rrt_planner/dual_arm_clearance_objective.h>
#include <dual_arm_rrt_planner/dual_arm_collision_interface.h>
#include <dual_arm_rrt_planner/dual_arm_heatmap_objective.h>
#include <dual_arm_rrt_planner/dual_arm_tree_visualizer.h>
#include <dual_arm_rrt_planner/heatmap_field.h>
#include <dual_arm_rrt_planner/local_trrt.h>
#include <dual_arm_rrt_planner/planner_config.h>

namespace dual_arm_rrt_planner
{
namespace
{
struct GoalRegionSpecification
{
  std::vector<double> nominal_joint_values;
  std::vector<double> lower_joint_values;
  std::vector<double> upper_joint_values;
  std::vector<bool> fixed_joint_mask;
  std::vector<double> fixed_joint_values;
  double success_distance_threshold{ 0.0 };
  bool left_arm_locked{ false };
  bool right_arm_locked{ false };
};

double computeGoalDistance(const std::vector<double>& joint_values, const GoalRegionSpecification& specification)
{
  double squared_violation_sum = 0.0;

  for (std::size_t index = 0; index < specification.lower_joint_values.size(); ++index)
  {
    double violation = 0.0;
    if (joint_values[index] < specification.lower_joint_values[index])
    {
      violation = specification.lower_joint_values[index] - joint_values[index];
    }
    else if (joint_values[index] > specification.upper_joint_values[index])
    {
      violation = joint_values[index] - specification.upper_joint_values[index];
    }

    squared_violation_sum += violation * violation;
  }

  return std::sqrt(squared_violation_sum);
}

double clampScalar(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

class JointConstraintGoalRegion : public ompl::base::GoalSampleableRegion
{
public:
  JointConstraintGoalRegion(const ompl::base::SpaceInformationPtr& space_information,
                            const GoalRegionSpecification& goal_region_specification)
    : ompl::base::GoalSampleableRegion(space_information), specification_(goal_region_specification)
  {
    setThreshold(specification_.success_distance_threshold);
  }

  double distanceGoal(const ompl::base::State* state) const override
  {
    const auto* joint_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> joint_values(specification_.lower_joint_values.size(), 0.0);
    for (std::size_t index = 0; index < specification_.lower_joint_values.size(); ++index)
    {
      joint_values[index] = joint_state->values[index];
    }

    return computeGoalDistance(joint_values, specification_);
  }

  void sampleGoal(ompl::base::State* state) const override
  {
    auto* joint_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
    for (std::size_t index = 0; index < specification_.lower_joint_values.size(); ++index)
    {
      joint_state->values[index] =
          rng_.uniformReal(specification_.lower_joint_values[index], specification_.upper_joint_values[index]);
    }
  }

  unsigned int maxSampleCount() const override
  {
    return std::numeric_limits<unsigned int>::max();
  }

private:
  GoalRegionSpecification specification_;
  mutable ompl::RNG rng_;
};

class DualArmTRRTPlanningContext : public planning_interface::PlanningContext
{
public:
  DualArmTRRTPlanningContext(const std::string& planner_name, const std::string& group_name,
                             const moveit::core::RobotModelConstPtr& robot_model, const PlannerConfig& config,
                             const std::shared_ptr<DualArmTreeVisualizer>& tree_visualizer,
                             const ros::NodeHandle& node_handle, const ros::Publisher& planner_stats_publisher)
    : planning_interface::PlanningContext(planner_name, group_name)
    , robot_model_(robot_model)
    , joint_model_group_(robot_model ? robot_model->getJointModelGroup(group_name) : nullptr)
    , left_arm_joint_model_group_(robot_model ? robot_model->getJointModelGroup(config.robot.left_arm_group) : nullptr)
    , right_arm_joint_model_group_(robot_model ? robot_model->getJointModelGroup(config.robot.right_arm_group) : nullptr)
    , config_(config)
    , tree_visualizer_(tree_visualizer)
    , planner_stats_publisher_(planner_stats_publisher)
    , node_handle_(node_handle)
    , heatmap_field_(config.heatmap.enabled ? std::make_shared<HeatmapField>(
                                                  node_handle_, config.heatmap.environment_topic,
                                                  config.heatmap.service_name)
                                            : nullptr)
    , terminate_requested_(false)
  {
    if (!robot_model_)
    {
      throw std::runtime_error("DualArmTRRTPlanningContext received a null robot model.");
    }

    if (!joint_model_group_)
    {
      throw std::runtime_error("Joint model group \"" + group_name + "\" does not exist.");
    }

    if (!left_arm_joint_model_group_)
    {
      throw std::runtime_error("Left arm joint model group \"" + config.robot.left_arm_group +
                               "\" does not exist.");
    }

    if (!right_arm_joint_model_group_)
    {
      throw std::runtime_error("Right arm joint model group \"" + config.robot.right_arm_group +
                               "\" does not exist.");
    }
  }

  bool solve(planning_interface::MotionPlanResponse& response) override
  {
    terminate_requested_.store(false);
    response = planning_interface::MotionPlanResponse();
    response.planner_id_ = config_.trrt.planner_config_name;

    if (tree_visualizer_ && !config_.visualization.publish_on_success && !config_.visualization.publish_on_failure)
    {
      tree_visualizer_->clear();
    }

    if (!getPlanningScene())
    {
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    moveit::core::RobotState start_robot_state(robot_model_);
    std::vector<double> start_joint_values;
    GoalRegionSpecification goal_region_specification;

    try
    {
      start_robot_state = resolveStartState();
      start_joint_values = extractJointValues(start_robot_state);
      goal_region_specification = resolveGoalRegionSpecification(start_joint_values);
    }
    catch (const std::exception& exception)
    {
      ROS_ERROR_STREAM("[DualArmTRRT] Request parsing failed: " << exception.what());
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    if (goal_region_specification.left_arm_locked)
    {
      ROS_INFO_STREAM("[DualArmTRRT] Left arm goal matches the start state. Locking left arm during planning.");
    }

    if (goal_region_specification.right_arm_locked)
    {
      ROS_INFO_STREAM("[DualArmTRRT] Right arm goal matches the start state. Locking right arm during planning.");
    }

    if (isStateWithinGoalRegion(start_joint_values, goal_region_specification))
    {
      response.trajectory_ = buildSingleStateTrajectory(start_robot_state);
      response.planning_time_ = 0.0;
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      moveit::core::robotStateToRobotStateMsg(start_robot_state, response.start_state_, false);
      ROS_INFO_STREAM("[DualArmTRRT] Start state already satisfies the goal region. Returning a trivial trajectory.");
      return true;
    }

    setupOmplInterfaces(goal_region_specification);
    prepareHeatmapField(start_robot_state, goal_region_specification.nominal_joint_values);

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start_state = makeScopedState(start_joint_values);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal_state =
        makeScopedState(goal_region_specification.nominal_joint_values);

    if (!config_.collision_checking.allowed_start_state_in_collision &&
        !collision_interface_->isStateValid(start_state.get()))
    {
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
      return false;
    }

    if (!config_.collision_checking.allowed_goal_state_in_collision &&
        !collision_interface_->isStateValid(goal_state.get()))
    {
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
      return false;
    }

    const int configured_planning_attempt_count = std::max(1, config_.solve.planning_attempts);
    const int requested_planning_attempt_count =
        request_.num_planning_attempts > 0 ? request_.num_planning_attempts : configured_planning_attempt_count;
    const int planning_attempt_count = std::max(1, requested_planning_attempt_count);
    const double configured_time_budget = config_.solve.max_planning_time_seconds;
    const double requested_time_budget =
        request_.allowed_planning_time > 0.0 ? request_.allowed_planning_time : configured_time_budget;
    const double total_time_budget = std::min(configured_time_budget, requested_time_budget);
    const double per_attempt_time_budget = total_time_budget / static_cast<double>(planning_attempt_count);

    ompl::base::PathPtr best_solution_path;
    std::shared_ptr<ompl::geometric::PathGeometric> best_solution_branch_path;
    double best_solution_cost = std::numeric_limits<double>::infinity();
    double best_solution_goal_difference = std::numeric_limits<double>::infinity();
    bool best_solution_is_approximate = false;
    std::string best_solution_path_label = "unknown";
    double accumulated_planning_time = 0.0;
    std::unique_ptr<ompl::base::PlannerData> selected_planner_data;
    bool selected_planner_data_success = false;
    int selected_planner_attempt_index = 0;
    bool last_failure_was_approximate_rejection = false;
    bool last_failure_was_dense_validation_rejection = false;
    double last_rejected_goal_difference = std::numeric_limits<double>::quiet_NaN();
    std::vector<std::size_t> last_dense_invalid_state_indices;
    bool observed_approximate_candidate = false;
    bool observed_dense_validation_rejection = false;

    if (request_.num_planning_attempts > 0 && request_.num_planning_attempts != configured_planning_attempt_count)
    {
      ROS_INFO_STREAM("[DualArmTRRT] Using request.num_planning_attempts=" << planning_attempt_count
                      << " instead of YAML solve/planning_attempts=" << configured_planning_attempt_count << ".");
    }

    const auto formatGoalViolationSummary = [&](const ompl::base::State* state) -> std::string {
      if (state == nullptr)
      {
        return "unavailable";
      }

      const auto* joint_state = state->as<ompl::base::RealVectorStateSpace::StateType>();
      if (joint_state == nullptr)
      {
        return "unavailable";
      }

      struct JointViolation
      {
        std::string name;
        double value;
        double lower;
        double upper;
        double violation;
      };

      std::vector<JointViolation> violations;
      for (std::size_t index = 0; index < config_.robot.joint_order.size() &&
                                  index < goal_region_specification.lower_joint_values.size();
           ++index)
      {
        const double value = joint_state->values[index];
        const double lower = goal_region_specification.lower_joint_values[index];
        const double upper = goal_region_specification.upper_joint_values[index];
        double violation = 0.0;
        if (value < lower)
        {
          violation = lower - value;
        }
        else if (value > upper)
        {
          violation = value - upper;
        }

        if (violation > 0.0)
        {
          violations.push_back({ config_.robot.joint_order[index], value, lower, upper, violation });
        }
      }

      if (violations.empty())
      {
        return "all joints within configured bounds";
      }

      std::sort(violations.begin(), violations.end(),
                [](const JointViolation& left, const JointViolation& right) { return left.violation > right.violation; });

      std::ostringstream summary;
      summary << std::fixed << std::setprecision(4);
      const std::size_t report_count = std::min<std::size_t>(violations.size(), 6U);
      for (std::size_t index = 0; index < report_count; ++index)
      {
        if (index > 0U)
        {
          summary << ", ";
        }

        const JointViolation& violation = violations[index];
        summary << violation.name << ": value=" << violation.value << ", goal=[" << violation.lower << ", "
                << violation.upper << "], excess=" << violation.violation;
      }

      if (violations.size() > report_count)
      {
        summary << ", ...";
      }

      return summary.str();
    };

    for (int attempt_index = 0; attempt_index < planning_attempt_count; ++attempt_index)
    {
      auto objective = createOptimizationObjective();
      auto problem_definition = std::make_shared<ompl::base::ProblemDefinition>(space_information_);
      problem_definition->setOptimizationObjective(objective);
      problem_definition->addStartState(start_state);

      auto goal_region =
          std::make_shared<JointConstraintGoalRegion>(space_information_, goal_region_specification);
      problem_definition->setGoal(goal_region);

      ompl::base::PlannerPtr planner = createPlanner(problem_definition, goal_state.get());
      const ros::WallTime attempt_start_time = ros::WallTime::now();

      const auto external_stop_condition =
          ompl::base::PlannerTerminationCondition([this]() { return terminate_requested_.load(); });
      const auto timed_stop_condition = ompl::base::timedPlannerTerminationCondition(per_attempt_time_budget);
      const auto combined_stop_condition =
          ompl::base::plannerOrTerminationCondition(timed_stop_condition, external_stop_condition);

      const ompl::base::PlannerStatus planner_status = planner->solve(combined_stop_condition);
      accumulated_planning_time += (ros::WallTime::now() - attempt_start_time).toSec();
      std::unique_ptr<ompl::base::PlannerData> attempt_planner_data = capturePlannerData(planner);

      if (!planner_status)
      {
        if (attempt_planner_data)
        {
          selected_planner_data = std::move(attempt_planner_data);
          selected_planner_data_success = false;
          selected_planner_attempt_index = attempt_index + 1;
        }

        if (terminate_requested_.load())
        {
          if (tree_visualizer_ && selected_planner_data)
          {
            tree_visualizer_->publishPlannerData(*selected_planner_data, config_.trrt.planner_config_name,
                                                 selected_planner_data_success, selected_planner_attempt_index, nullptr,
                                                 nullptr, !goal_region_specification.left_arm_locked,
                                                 !goal_region_specification.right_arm_locked);
          }
          publishPlannerStats(selected_planner_data.get(), selected_planner_data_success, selected_planner_attempt_index);
          response.error_code_.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
          return false;
        }
        continue;
      }

      bool planner_returned_approximate =
          planner_status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION ||
          (problem_definition->hasApproximateSolution() && !problem_definition->hasExactSolution());
      double candidate_goal_difference =
          planner_returned_approximate ? problem_definition->getSolutionDifference() : 0.0;
      std::shared_ptr<ompl::geometric::PathGeometric> candidate_geometric_path =
          std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problem_definition->getSolutionPath());
      const auto tryRepairApproximateTail = [&](ompl::geometric::PathGeometric& path,
                                                double approximate_goal_difference) -> bool {
        if (!space_information_ || path.getStateCount() == 0U || goal_region == nullptr)
        {
          return false;
        }

        const double repair_goal_difference_threshold = std::max(0.25, 5.0 * config_.solve.goal_tolerance);
        if (!std::isfinite(approximate_goal_difference) ||
            approximate_goal_difference > repair_goal_difference_threshold)
        {
          return false;
        }

        const ompl::base::State* tail_state = path.getState(path.getStateCount() - 1U);
        if (tail_state == nullptr)
        {
          return false;
        }

        ompl::base::State* sampled_goal_state = space_information_->allocState();
        auto free_sampled_goal_state = [this, &sampled_goal_state]() {
          if (sampled_goal_state != nullptr)
          {
            space_information_->freeState(sampled_goal_state);
            sampled_goal_state = nullptr;
          }
        };

        const auto tryAppendGoalState = [&](const ompl::base::State* candidate_goal_state) -> bool {
          if (candidate_goal_state == nullptr)
          {
            return false;
          }

          double distance_to_goal = std::numeric_limits<double>::infinity();
          if (!goal_region->isSatisfied(candidate_goal_state, &distance_to_goal))
          {
            return false;
          }

          if (!space_information_->checkMotion(tail_state, candidate_goal_state))
          {
            return false;
          }

          path.append(candidate_goal_state);
          return true;
        };

        if (tryAppendGoalState(goal_state.get()))
        {
          free_sampled_goal_state();
          return true;
        }

        constexpr unsigned int kGoalRepairSampleCount = 24U;
        for (unsigned int sample_index = 0U; sample_index < kGoalRepairSampleCount; ++sample_index)
        {
          goal_region->sampleGoal(sampled_goal_state);
          if (tryAppendGoalState(sampled_goal_state))
          {
            free_sampled_goal_state();
            return true;
          }
        }

        free_sampled_goal_state();
        auto polish_problem_definition = std::make_shared<ompl::base::ProblemDefinition>(space_information_);
        polish_problem_definition->setOptimizationObjective(objective);
        polish_problem_definition->addStartState(tail_state);
        auto polish_goal_region =
            std::make_shared<JointConstraintGoalRegion>(space_information_, goal_region_specification);
        polish_problem_definition->setGoal(polish_goal_region);

        ompl::base::PlannerPtr polish_planner = createPlanner(polish_problem_definition, goal_state.get());
        const double polish_time_budget = std::min(0.20, std::max(0.05, 0.10 * per_attempt_time_budget));
        const auto polish_stop_condition = ompl::base::plannerOrTerminationCondition(
            ompl::base::timedPlannerTerminationCondition(polish_time_budget),
            ompl::base::PlannerTerminationCondition([this]() { return terminate_requested_.load(); }));
        const ompl::base::PlannerStatus polish_status = polish_planner->solve(polish_stop_condition);
        const bool polish_exact =
            polish_status == ompl::base::PlannerStatus::EXACT_SOLUTION || polish_problem_definition->hasExactSolution();
        if (!polish_exact)
        {
          return false;
        }

        auto polish_path =
            std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(polish_problem_definition->getSolutionPath());
        if (!polish_path || polish_path->getStateCount() < 2U)
        {
          return false;
        }

        for (std::size_t state_index = 1U; state_index < polish_path->getStateCount(); ++state_index)
        {
          path.append(polish_path->getState(state_index));
        }
        return true;
      };
      if (planner_returned_approximate && candidate_geometric_path &&
          tryRepairApproximateTail(*candidate_geometric_path, candidate_goal_difference))
      {
        planner_returned_approximate = false;
        candidate_goal_difference = 0.0;
        ROS_WARN_STREAM("[DualArmTRRT] Repaired an approximate solution by locally connecting the final state into "
                        "the goal region for attempt " << attempt_index + 1 << ".");
      }

      const bool accept_approximate_solution =
          planner_returned_approximate && config_.solve.allow_approximate_solution &&
          std::isfinite(candidate_goal_difference) &&
          candidate_goal_difference <= config_.solve.max_approximate_goal_distance;

      if (planner_returned_approximate && !accept_approximate_solution)
      {
        observed_approximate_candidate = true;
        const ompl::base::State* approximate_tail_state =
            (candidate_geometric_path && candidate_geometric_path->getStateCount() > 0U)
                ? candidate_geometric_path->getState(candidate_geometric_path->getStateCount() - 1U)
                : nullptr;
        ROS_WARN_STREAM("[DualArmTRRT] OMPL returned an approximate solution with goal difference "
                        << candidate_goal_difference
                        << ". Rejecting it because it exceeds solve/max_approximate_goal_distance="
                        << config_.solve.max_approximate_goal_distance
                        << " or approximate acceptance is disabled. Joint goal violations: "
                        << formatGoalViolationSummary(approximate_tail_state) << ".");
        last_failure_was_approximate_rejection = true;
        last_failure_was_dense_validation_rejection = false;
        last_rejected_goal_difference = candidate_goal_difference;
        last_dense_invalid_state_indices.clear();
        if (attempt_planner_data)
        {
          selected_planner_data = std::move(attempt_planner_data);
          selected_planner_data_success = false;
          selected_planner_attempt_index = attempt_index + 1;
        }
        continue;
      }

      if (accept_approximate_solution)
      {
        ROS_WARN_STREAM("[DualArmTRRT] Accepting approximate solution with goal difference "
                        << candidate_goal_difference << " because it is within solve/max_approximate_goal_distance="
                        << config_.solve.max_approximate_goal_distance << ".");
      }

      if (!candidate_geometric_path)
      {
        if (attempt_planner_data)
        {
          selected_planner_data = std::move(attempt_planner_data);
          selected_planner_data_success = false;
          selected_planner_attempt_index = attempt_index + 1;
        }
        continue;
      }

      std::vector<std::size_t> invalid_state_indices;
      std::vector<std::size_t> invalid_segment_indices;
      if (!validateGeometricPathStates(*candidate_geometric_path, &invalid_state_indices))
      {
        observed_dense_validation_rejection = true;
        ROS_WARN_STREAM("[DualArmTRRT] Raw planner path contains invalid states at indices "
                        << formatIndexList(invalid_state_indices) << ". Discarding attempt "
                        << attempt_index + 1 << ".");
        last_failure_was_approximate_rejection = false;
        last_failure_was_dense_validation_rejection = true;
        last_rejected_goal_difference = std::numeric_limits<double>::quiet_NaN();
        last_dense_invalid_state_indices = invalid_state_indices;
        if (attempt_planner_data)
        {
          selected_planner_data = std::move(attempt_planner_data);
          selected_planner_data_success = false;
          selected_planner_attempt_index = attempt_index + 1;
        }
        continue;
      }

      if (!validateGeometricPathMotions(*candidate_geometric_path, &invalid_segment_indices))
      {
        observed_dense_validation_rejection = true;
        ROS_WARN_STREAM("[DualArmTRRT] Raw planner path contains invalid motions starting at indices "
                        << formatIndexList(invalid_segment_indices) << ". Discarding attempt "
                        << attempt_index + 1 << ".");
        last_failure_was_approximate_rejection = false;
        last_failure_was_dense_validation_rejection = true;
        last_rejected_goal_difference = std::numeric_limits<double>::quiet_NaN();
        last_dense_invalid_state_indices = invalid_segment_indices;
        if (attempt_planner_data)
        {
          selected_planner_data = std::move(attempt_planner_data);
          selected_planner_data_success = false;
          selected_planner_attempt_index = attempt_index + 1;
        }
        continue;
      }

      std::shared_ptr<ompl::geometric::PathGeometric> candidate_solution_branch_path =
          std::make_shared<ompl::geometric::PathGeometric>(*candidate_geometric_path);
      std::shared_ptr<ompl::geometric::PathGeometric> processed_geometric_path =
          std::make_shared<ompl::geometric::PathGeometric>(*candidate_geometric_path);
      std::shared_ptr<ompl::geometric::PathGeometric> pre_interpolation_geometric_path =
          std::make_shared<ompl::geometric::PathGeometric>(*candidate_geometric_path);
      std::string processed_path_label = "raw planner path";
      std::string pre_interpolation_path_label = processed_path_label;
      bool using_simplified_path = false;
      bool using_interpolated_path = false;
      const double dense_validation_step = computeDenseValidationStep();
      const auto makeAnnealedCandidate =
          [&](const std::shared_ptr<ompl::geometric::PathGeometric>& source_path,
              const std::function<void(ompl::geometric::PathGeometric&, double)>& simplify_operation)
          -> std::shared_ptr<ompl::geometric::PathGeometric> {
        if (!source_path)
        {
          return nullptr;
        }

        auto working_candidate = std::make_shared<ompl::geometric::PathGeometric>(*source_path);
        if (working_candidate->getStateCount() < 4U)
        {
          simplify_operation(*working_candidate, 1.0);
          return working_candidate;
        }

        constexpr std::size_t kAnnealStageCount = 5U;
        constexpr double kAnnealMinScale = 0.35;
        constexpr double kAnnealExponent = 1.2;
        const double final_preserve_ratio = clampScalar(config_.solve.preserve_tail_ratio, 0.0, 0.30);
        const double initial_preserve_ratio = clampScalar(std::max(final_preserve_ratio * 2.0,
                                                                   final_preserve_ratio + 0.05),
                                                          final_preserve_ratio, 0.18);

        for (std::size_t stage_index = 0; stage_index < kAnnealStageCount; ++stage_index)
        {
          const double progress = kAnnealStageCount > 1U
                                      ? static_cast<double>(stage_index) / static_cast<double>(kAnnealStageCount - 1U)
                                      : 1.0;
          const double decay = std::pow(1.0 - progress, kAnnealExponent);
          const double stage_scale = kAnnealMinScale + (1.0 - kAnnealMinScale) * decay;
          const double stage_preserve_ratio =
              final_preserve_ratio + (initial_preserve_ratio - final_preserve_ratio) * decay;

          const std::size_t state_count = working_candidate->getStateCount();
          std::size_t preserved_tail_state_count =
              static_cast<std::size_t>(std::ceil(static_cast<double>(state_count) * stage_preserve_ratio));
          preserved_tail_state_count = std::min<std::size_t>(preserved_tail_state_count, state_count - 2U);
          const std::size_t prefix_end_index = state_count - preserved_tail_state_count - 1U;
          if (prefix_end_index < 1U)
          {
            continue;
          }

          auto simplified_prefix = std::make_shared<ompl::geometric::PathGeometric>(space_information_);
          for (std::size_t state_index = 0; state_index <= prefix_end_index; ++state_index)
          {
            simplified_prefix->append(working_candidate->getState(state_index));
          }

          simplify_operation(*simplified_prefix, stage_scale);

          auto combined_path = std::make_shared<ompl::geometric::PathGeometric>(space_information_);
          for (std::size_t state_index = 0; state_index < simplified_prefix->getStateCount(); ++state_index)
          {
            combined_path->append(simplified_prefix->getState(state_index));
          }
          for (std::size_t state_index = prefix_end_index + 1U; state_index < state_count; ++state_index)
          {
            combined_path->append(working_candidate->getState(state_index));
          }

          working_candidate = combined_path;
        }

        return working_candidate;
      };

      if (config_.solve.simplify_solution)
      {
        ompl::geometric::PathSimplifier path_simplifier(space_information_, goal_region, objective);

        if (config_.solve.simplification_strategy == "simplify_max")
        {
          auto simplified_path = makeAnnealedCandidate(
              processed_geometric_path,
              [&](ompl::geometric::PathGeometric& path, double /*stage_scale*/) { path_simplifier.simplifyMax(path); });
          if (simplified_path &&
              tryAdoptValidatedPostProcessedPath(*simplified_path, "simplifyMax path", processed_path_label,
                                                 attempt_index, dense_validation_step, processed_geometric_path,
                                                 processed_path_label))
          {
            using_simplified_path = true;
          }
        }
        else
        {
          if (config_.solve.simplification_strategy == "reduce_vertices" ||
              config_.solve.simplification_strategy == "reduce_shortcut")
          {
            auto reduced_path = makeAnnealedCandidate(
                processed_geometric_path,
                [&](ompl::geometric::PathGeometric& path, double stage_scale) {
                  const unsigned int stage_max_steps =
                      std::max(1U, static_cast<unsigned int>(std::lround(config_.solve.reduce_vertices_max_steps *
                                                                          stage_scale)));
                  const unsigned int stage_max_empty_steps =
                      std::max(1U, static_cast<unsigned int>(std::lround(config_.solve.reduce_vertices_max_empty_steps *
                                                                          stage_scale)));
                  const double stage_range_ratio =
                      clampScalar(config_.solve.reduce_vertices_range_ratio * stage_scale, 0.01, 1.0);
                  path_simplifier.reduceVertices(path, stage_max_steps, stage_max_empty_steps, stage_range_ratio);
                });
            if (reduced_path &&
                tryAdoptValidatedPostProcessedPath(*reduced_path, "reduced-vertices path", processed_path_label,
                                                   attempt_index, dense_validation_step, processed_geometric_path,
                                                   processed_path_label))
            {
              using_simplified_path = true;
            }
          }

          if (config_.solve.simplification_strategy == "shortcut" ||
              config_.solve.simplification_strategy == "reduce_shortcut")
          {
            auto shortcut_path = makeAnnealedCandidate(
                processed_geometric_path,
                [&](ompl::geometric::PathGeometric& path, double stage_scale) {
                  const unsigned int stage_max_steps =
                      std::max(1U, static_cast<unsigned int>(std::lround(config_.solve.shortcut_max_steps *
                                                                          stage_scale)));
                  const unsigned int stage_max_empty_steps =
                      std::max(1U, static_cast<unsigned int>(std::lround(config_.solve.shortcut_max_empty_steps *
                                                                          stage_scale)));
                  const double stage_range_ratio =
                      clampScalar(config_.solve.shortcut_range_ratio * stage_scale, 0.01, 1.0);
                  path_simplifier.shortcutPath(path, stage_max_steps, stage_max_empty_steps, stage_range_ratio,
                                               config_.solve.shortcut_snap_to_vertex);
                });
            if (shortcut_path &&
                tryAdoptValidatedPostProcessedPath(*shortcut_path, "shortcut path", processed_path_label,
                                                   attempt_index, dense_validation_step, processed_geometric_path,
                                                   processed_path_label))
            {
              using_simplified_path = true;
            }
          }

          if (config_.solve.smooth_bspline)
          {
            auto smoothed_path = makeAnnealedCandidate(
                processed_geometric_path,
                [&](ompl::geometric::PathGeometric& path, double stage_scale) {
                  path_simplifier.smoothBSpline(path, static_cast<unsigned int>(config_.solve.smooth_bspline_max_steps),
                                                config_.solve.smooth_bspline_min_change * stage_scale);
                });
            if (smoothed_path &&
                tryAdoptValidatedPostProcessedPath(*smoothed_path, "B-spline-smoothed path", processed_path_label,
                                                   attempt_index, dense_validation_step, processed_geometric_path,
                                                   processed_path_label))
            {
              using_simplified_path = true;
            }
          }
        }

        pre_interpolation_geometric_path =
            std::make_shared<ompl::geometric::PathGeometric>(*processed_geometric_path);
        pre_interpolation_path_label = processed_path_label;
      }
      if (config_.solve.interpolate_solution)
      {
        auto interpolated_path = std::make_shared<ompl::geometric::PathGeometric>(*processed_geometric_path);
        const unsigned int interpolated_state_count =
            std::max(2U, static_cast<unsigned int>(
                             std::ceil(interpolated_path->length() / config_.solve.interpolation_step)) +
                             1U);
        interpolated_path->interpolate(interpolated_state_count);

        const bool interpolated_states_valid =
            validateGeometricPathStates(*interpolated_path, &invalid_state_indices);
        const bool interpolated_motions_valid =
            validateGeometricPathMotions(*interpolated_path, &invalid_segment_indices);
        if (!interpolated_states_valid || !interpolated_motions_valid)
        {
          ROS_WARN_STREAM("[DualArmTRRT] Interpolated path became invalid. Invalid states="
                          << formatIndexList(invalid_state_indices) << ", invalid motions="
                          << formatIndexList(invalid_segment_indices) << ". Falling back to the "
                          << pre_interpolation_path_label << " for attempt " << attempt_index + 1 << ".");
          processed_geometric_path =
              std::make_shared<ompl::geometric::PathGeometric>(*pre_interpolation_geometric_path);
          processed_path_label = pre_interpolation_path_label;
        }
        else
        {
          processed_geometric_path = interpolated_path;
          processed_path_label = "interpolated path";
          using_interpolated_path = true;
        }
      }

      if (!validateGeometricPathDensely(*processed_geometric_path, &invalid_state_indices, dense_validation_step))
      {
        observed_dense_validation_rejection = true;
        std::ostringstream step_stream;
        step_stream << std::fixed << std::setprecision(4) << dense_validation_step;
        if (using_interpolated_path)
        {
          ROS_WARN_STREAM("[DualArmTRRT] Interpolated path failed dense validation at sampled states "
                          << formatIndexList(invalid_state_indices) << " with validation step "
                          << step_stream.str() << ". Falling back to the " << pre_interpolation_path_label
                          << " for attempt " << attempt_index + 1 << ".");
          processed_geometric_path =
              std::make_shared<ompl::geometric::PathGeometric>(*pre_interpolation_geometric_path);
          processed_path_label = pre_interpolation_path_label;
          using_interpolated_path = false;
        }
        else if (using_simplified_path)
        {
          ROS_WARN_STREAM("[DualArmTRRT] " << processed_path_label
                          << " failed dense validation at sampled states "
                          << formatIndexList(invalid_state_indices) << " with validation step "
                          << step_stream.str() << ". Falling back to the raw planner path for attempt "
                          << attempt_index + 1 << ".");
          processed_geometric_path =
              std::make_shared<ompl::geometric::PathGeometric>(*candidate_solution_branch_path);
          processed_path_label = "raw planner path";
          using_simplified_path = false;
        }
        else
        {
          ROS_WARN_STREAM("[DualArmTRRT] Raw planner path failed dense validation at sampled states "
                          << formatIndexList(invalid_state_indices) << " with validation step "
                          << step_stream.str() << ". Discarding attempt " << attempt_index + 1
                          << " because the raw path is not safe under dense validation.");
          last_failure_was_approximate_rejection = false;
          last_failure_was_dense_validation_rejection = true;
          last_rejected_goal_difference = std::numeric_limits<double>::quiet_NaN();
          last_dense_invalid_state_indices = invalid_state_indices;
          if (attempt_planner_data)
          {
            selected_planner_data = std::move(attempt_planner_data);
            selected_planner_data_success = false;
            selected_planner_attempt_index = attempt_index + 1;
          }
          continue;
        }
      }

      const double candidate_cost = processed_geometric_path->cost(objective).value();
      const bool candidate_is_approximate = accept_approximate_solution;
      const bool candidate_is_better =
          !best_solution_path ||
          (best_solution_is_approximate && !candidate_is_approximate) ||
          (best_solution_is_approximate == candidate_is_approximate &&
           ((candidate_is_approximate &&
             (candidate_goal_difference < best_solution_goal_difference ||
              (candidate_goal_difference == best_solution_goal_difference && candidate_cost < best_solution_cost))) ||
            (!candidate_is_approximate && candidate_cost < best_solution_cost)));

      if (candidate_is_better)
      {
        best_solution_cost = candidate_cost;
        best_solution_goal_difference = candidate_goal_difference;
        best_solution_is_approximate = candidate_is_approximate;
        best_solution_path_label = processed_path_label;
        best_solution_path = processed_geometric_path;
        best_solution_branch_path = candidate_solution_branch_path;
        if (attempt_planner_data)
        {
          selected_planner_data = std::move(attempt_planner_data);
          selected_planner_data_success = true;
          selected_planner_attempt_index = attempt_index + 1;
        }
      }
    }

    if (!best_solution_path)
    {
      if (last_failure_was_approximate_rejection)
      {
        ROS_WARN_STREAM("[DualArmTRRT] No highlighted solution was published because the best candidate remained an "
                        "approximate solution with goal difference " << last_rejected_goal_difference << ".");
      }
      else if (last_failure_was_dense_validation_rejection)
      {
        ROS_WARN_STREAM("[DualArmTRRT] No highlighted solution was published because the best candidate failed dense "
                        "validation near sampled indices " << formatIndexList(last_dense_invalid_state_indices) << ".");
      }

      if (tree_visualizer_ && selected_planner_data)
      {
        tree_visualizer_->publishPlannerData(*selected_planner_data, config_.trrt.planner_config_name,
                                             selected_planner_data_success, selected_planner_attempt_index, nullptr,
                                             nullptr, !goal_region_specification.left_arm_locked,
                                             !goal_region_specification.right_arm_locked);
      }
      publishPlannerStats(selected_planner_data.get(), selected_planner_data_success, selected_planner_attempt_index);
      ROS_WARN_STREAM("[DualArmTRRT] Final result: planning failed. Approximate candidate observed="
                      << (observed_approximate_candidate ? "true" : "false")
                      << ", dense-validation rejection observed="
                      << (observed_dense_validation_rejection ? "true" : "false") << ".");
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    std::shared_ptr<ompl::geometric::PathGeometric> geometric_path =
        std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(best_solution_path);
    if (!geometric_path)
    {
      response.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
      return false;
    }

    response.trajectory_ = buildRobotTrajectory(*geometric_path, start_robot_state);
    response.planning_time_ = accumulated_planning_time;
    response.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    moveit::core::robotStateToRobotStateMsg(start_robot_state, response.start_state_, false);

    if (tree_visualizer_ && selected_planner_data)
    {
      tree_visualizer_->publishPlannerData(*selected_planner_data, config_.trrt.planner_config_name,
                                           selected_planner_data_success, selected_planner_attempt_index,
                                           best_solution_branch_path.get(), geometric_path.get(),
                                           !goal_region_specification.left_arm_locked,
                                           !goal_region_specification.right_arm_locked);
    }
    const bool highlight_published =
        tree_visualizer_ &&
        tree_visualizer_->publishSolutionPathOnly(config_.trrt.planner_config_name, *geometric_path,
                                                  !goal_region_specification.left_arm_locked,
                                                  !goal_region_specification.right_arm_locked);
    publishPlannerStats(selected_planner_data.get(), selected_planner_data_success, selected_planner_attempt_index,
                        geometric_path.get());
    ROS_INFO_STREAM("[DualArmTRRT] Final result: exact success. Final trajectory highlight "
                    << (highlight_published ? "published" : "not published")
                    << ". Selected path source=" << best_solution_path_label
                    << ". Approximate candidate observed=" << (observed_approximate_candidate ? "true" : "false")
                    << ", dense-validation rejection observed="
                    << (observed_dense_validation_rejection ? "true" : "false") << ".");

    ROS_INFO_STREAM("[DualArmTRRT] " << config_.trrt.planner_config_name << " solved with "
                                     << geometric_path->getStateCount() << " states, path length "
                                     << geometric_path->length() << ", time " << accumulated_planning_time
                                     << " s, cost " << best_solution_cost
                                     << (best_solution_is_approximate
                                             ? ", accepted approximate goal difference " +
                                                   std::to_string(best_solution_goal_difference)
                                             : ", exact goal satisfaction")
                                     << ".");
    return true;
  }

  bool solve(planning_interface::MotionPlanDetailedResponse& response) override
  {
    planning_interface::MotionPlanResponse simple_response;
    const bool success = solve(simple_response);
    response.error_code_ = simple_response.error_code_;
    response.start_state_ = simple_response.start_state_;
    response.planner_id_ = simple_response.planner_id_;
    if (success)
    {
      response.trajectory_.push_back(simple_response.trajectory_);
      response.processing_time_.push_back(simple_response.planning_time_);
      response.description_.push_back(config_.trrt.planner_config_name);
    }
    return success;
  }

  bool terminate() override
  {
    terminate_requested_.store(true);
    return true;
  }

  void clear() override
  {
    terminate_requested_.store(false);
    collision_interface_.reset();
    space_information_.reset();
    state_space_.reset();
  }

private:
  std::string formatIndexList(const std::vector<std::size_t>& indices) const
  {
    if (indices.empty())
    {
      return "[]";
    }

    std::ostringstream stream;
    stream << "[";
    for (std::size_t index = 0; index < indices.size(); ++index)
    {
      if (index > 0U)
      {
        stream << ", ";
      }
      stream << indices[index];
    }
    stream << "]";
    return stream.str();
  }

  bool validateGeometricPathStates(const ompl::geometric::PathGeometric& path,
                                   std::vector<std::size_t>* invalid_state_indices,
                                   std::size_t max_record_count = 8U) const
  {
    if (invalid_state_indices)
    {
      invalid_state_indices->clear();
    }

    if (!collision_interface_ || path.getStateCount() == 0U)
    {
      return false;
    }

    bool path_is_valid = true;
    for (std::size_t state_index = 0; state_index < path.getStateCount(); ++state_index)
    {
      if (collision_interface_->isStateValid(path.getState(state_index)))
      {
        continue;
      }

      path_is_valid = false;
      if (invalid_state_indices && invalid_state_indices->size() < max_record_count)
      {
        invalid_state_indices->push_back(state_index);
      }
    }

    return path_is_valid;
  }

  bool validateGeometricPathMotions(const ompl::geometric::PathGeometric& path,
                                    std::vector<std::size_t>* invalid_segment_indices,
                                    std::size_t max_record_count = 8U) const
  {
    if (invalid_segment_indices)
    {
      invalid_segment_indices->clear();
    }

    if (!space_information_ || path.getStateCount() == 0U)
    {
      return false;
    }

    bool path_is_valid = true;
    for (std::size_t state_index = 1U; state_index < path.getStateCount(); ++state_index)
    {
      if (space_information_->checkMotion(path.getState(state_index - 1U), path.getState(state_index)))
      {
        continue;
      }

      path_is_valid = false;
      if (invalid_segment_indices && invalid_segment_indices->size() < max_record_count)
      {
        invalid_segment_indices->push_back(state_index - 1U);
      }
    }

    return path_is_valid;
  }

  bool validateGeometricPathDensely(const ompl::geometric::PathGeometric& path,
                                    std::vector<std::size_t>* invalid_state_indices,
                                    double validation_step) const
  {
    if (path.getStateCount() == 0U)
    {
      if (invalid_state_indices)
      {
        invalid_state_indices->clear();
      }
      return false;
    }

    if (!(validation_step > 0.0) || !std::isfinite(validation_step))
    {
      return validateGeometricPathStates(path, invalid_state_indices);
    }

    ompl::geometric::PathGeometric dense_path(path);
    const unsigned int interpolated_state_count =
        std::max(2U, static_cast<unsigned int>(std::ceil(dense_path.length() / validation_step)) + 1U);
    dense_path.interpolate(interpolated_state_count);
    return validateGeometricPathStates(dense_path, invalid_state_indices);
  }

  double computeDenseValidationStep() const
  {
    if (!space_information_)
    {
      return config_.solve.interpolation_step;
    }

    return std::min(config_.solve.interpolation_step,
                    config_.collision_checking.state_validity_checking_resolution *
                        space_information_->getMaximumExtent());
  }

  bool tryAdoptValidatedPostProcessedPath(const ompl::geometric::PathGeometric& candidate_path,
                                          const std::string& candidate_label,
                                          const std::string& previous_label, int attempt_index,
                                          double dense_validation_step,
                                          std::shared_ptr<ompl::geometric::PathGeometric>& accepted_path,
                                          std::string& accepted_path_label) const
  {
    std::vector<std::size_t> invalid_state_indices;
    std::vector<std::size_t> invalid_segment_indices;
    const bool candidate_states_valid = validateGeometricPathStates(candidate_path, &invalid_state_indices);
    const bool candidate_motions_valid = validateGeometricPathMotions(candidate_path, &invalid_segment_indices);

    if (!candidate_states_valid || !candidate_motions_valid)
    {
      ROS_WARN_STREAM("[DualArmTRRT] " << candidate_label << " became invalid. Invalid states="
                                       << formatIndexList(invalid_state_indices) << ", invalid motions="
                                       << formatIndexList(invalid_segment_indices) << ". Falling back to the "
                                       << previous_label << " for attempt " << attempt_index + 1 << ".");
      return false;
    }

    if (!validateGeometricPathDensely(candidate_path, &invalid_state_indices, dense_validation_step))
    {
      std::ostringstream step_stream;
      step_stream << std::fixed << std::setprecision(4) << dense_validation_step;
      ROS_WARN_STREAM("[DualArmTRRT] " << candidate_label
                                       << " failed dense validation at sampled states "
                                       << formatIndexList(invalid_state_indices) << " with validation step "
                                       << step_stream.str() << ". Falling back to the " << previous_label
                                       << " for attempt " << attempt_index + 1 << ".");
      return false;
    }

    accepted_path = std::make_shared<ompl::geometric::PathGeometric>(candidate_path);
    accepted_path_label = candidate_label;
    return true;
  }

  std::vector<Eigen::Vector3d> extractHeatmapQueryPoints(const moveit::core::RobotState& robot_state) const
  {
    std::vector<Eigen::Vector3d> query_points;
    query_points.reserve(2U);

    const auto append_link_point = [&](const std::string& link_name) {
      if (link_name.empty())
      {
        return;
      }

      if (!robot_model_->getLinkModel(link_name))
      {
        ROS_WARN_STREAM_THROTTLE(2.0, "[DualArmTRRT] Heatmap query link " << link_name << " does not exist.");
        return;
      }

      query_points.emplace_back(robot_state.getGlobalLinkTransform(link_name).translation());
    };

    append_link_point(config_.heatmap.left_query_link);
    if (config_.heatmap.right_query_link != config_.heatmap.left_query_link)
    {
      append_link_point(config_.heatmap.right_query_link);
    }

    return query_points;
  }

  bool computeHeatmapRequestBounds(const moveit::core::RobotState& start_robot_state,
                                   const std::vector<double>& goal_joint_values, Eigen::Vector3d& min_bound,
                                   Eigen::Vector3d& max_bound) const
  {
    bool has_bounds = false;
    const auto include_point = [&](const Eigen::Vector3d& point) {
      if (!point.allFinite())
      {
        return;
      }

      if (!has_bounds)
      {
        min_bound = point;
        max_bound = point;
        has_bounds = true;
        return;
      }

      min_bound = min_bound.cwiseMin(point);
      max_bound = max_bound.cwiseMax(point);
    };

    double table_z = std::numeric_limits<double>::quiet_NaN();
    if (heatmap_field_ && heatmap_field_->hasEnvironmentInfo())
    {
      const auto environment_info = heatmap_field_->latestEnvironmentInfo();
      table_z = static_cast<double>(environment_info.table_z);

      for (const auto& obstacle : environment_info.obstacles)
      {
        const Eigen::Vector3d center(obstacle.center.x, obstacle.center.y, obstacle.center.z);
        const Eigen::Vector3d half_size(0.5 * obstacle.size.x, 0.5 * obstacle.size.y, 0.5 * obstacle.size.z);
        include_point(center - half_size);
        include_point(center + half_size);
      }
    }

    for (const auto& point : extractHeatmapQueryPoints(start_robot_state))
    {
      include_point(point);
    }

    moveit::core::RobotState goal_robot_state(start_robot_state);
    for (std::size_t index = 0; index < config_.robot.joint_order.size() && index < goal_joint_values.size(); ++index)
    {
      goal_robot_state.setVariablePosition(config_.robot.joint_order[index], goal_joint_values[index]);
    }
    goal_robot_state.update();

    for (const auto& point : extractHeatmapQueryPoints(goal_robot_state))
    {
      include_point(point);
    }

    if (!has_bounds)
    {
      return false;
    }

    if (std::isfinite(table_z))
    {
      min_bound.z() = std::min(min_bound.z(), table_z);
    }

    min_bound.array() -= config_.heatmap.workspace_padding;
    max_bound.array() += config_.heatmap.workspace_padding;

    for (int axis = 0; axis < 3; ++axis)
    {
      if (max_bound[axis] - min_bound[axis] >= config_.heatmap.resolution)
      {
        continue;
      }

      const double center = 0.5 * (min_bound[axis] + max_bound[axis]);
      min_bound[axis] = center - 0.5 * config_.heatmap.resolution;
      max_bound[axis] = center + 0.5 * config_.heatmap.resolution;
    }

    return true;
  }

  bool prepareHeatmapField(const moveit::core::RobotState& start_robot_state,
                           const std::vector<double>& goal_joint_values)
  {
    heatmap_ready_ = false;
    if (!config_.heatmap.enabled || !heatmap_field_)
    {
      return false;
    }

    Eigen::Vector3d min_bound = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bound = Eigen::Vector3d::Zero();
    if (!computeHeatmapRequestBounds(start_robot_state, goal_joint_values, min_bound, max_bound))
    {
      ROS_WARN_STREAM("[DualArmTRRT] Unable to infer heatmap request bounds. Continuing without heatmap guidance.");
      return false;
    }

    std::string error_message;
    if (!heatmap_field_->requestField(config_.heatmap.resolution, min_bound, max_bound,
                                      config_.heatmap.request_timeout_seconds, &error_message))
    {
      ROS_WARN_STREAM("[DualArmTRRT] Heatmap request failed: " << error_message
                                                               << ". Continuing without heatmap guidance.");
      return false;
    }

    heatmap_ready_ = true;
    const auto grid = heatmap_field_->grid();
    ROS_INFO_STREAM("[DualArmTRRT] Heatmap ready: " << grid.dim_x << " x " << grid.dim_y << " x " << grid.dim_z
                                                    << " @ " << grid.resolution << " m.");
    return true;
  }

  double evaluateHeatmapClearance(const ompl::base::State* state) const
  {
    if (!heatmap_ready_ || !heatmap_field_ || !collision_interface_)
    {
      return config_.heatmap.safety_clearance_high;
    }

    const moveit::core::RobotState robot_state = collision_interface_->toRobotState(state);
    const auto query_points = extractHeatmapQueryPoints(robot_state);
    if (query_points.empty())
    {
      return config_.heatmap.safety_clearance_high;
    }

    double minimum_clearance = std::numeric_limits<double>::infinity();
    for (const auto& point : query_points)
    {
      minimum_clearance = std::min(minimum_clearance, heatmap_field_->valueAt(point));
    }

    if (!std::isfinite(minimum_clearance))
    {
      return config_.heatmap.safety_clearance_high;
    }

    return minimum_clearance;
  }

  double evaluateHeatmapSafety(const ompl::base::State* state) const
  {
    const double heatmap_clearance = evaluateHeatmapClearance(state);
    if (heatmap_clearance <= config_.heatmap.safety_clearance_low)
    {
      return 0.0;
    }

    if (heatmap_clearance >= config_.heatmap.safety_clearance_high)
    {
      return 1.0;
    }

    return (heatmap_clearance - config_.heatmap.safety_clearance_low) /
           (config_.heatmap.safety_clearance_high - config_.heatmap.safety_clearance_low);
  }

  void setupOmplInterfaces(const GoalRegionSpecification& goal_region_specification)
  {
    auto real_vector_state_space =
        std::make_shared<ompl::base::RealVectorStateSpace>(config_.robot.configuration_dimension);
    ompl::base::RealVectorBounds bounds(config_.robot.configuration_dimension);

    for (std::size_t index = 0; index < config_.robot.joint_order.size(); ++index)
    {
      const JointBoundConfig& joint_bound = config_.state_space.joint_bounds.at(config_.robot.joint_order[index]);
      if (index < goal_region_specification.fixed_joint_mask.size() && goal_region_specification.fixed_joint_mask[index])
      {
        bounds.setLow(index, goal_region_specification.fixed_joint_values[index]);
        bounds.setHigh(index, goal_region_specification.fixed_joint_values[index]);
      }
      else
      {
        bounds.setLow(index, joint_bound.lower);
        bounds.setHigh(index, joint_bound.upper);
      }
    }

    real_vector_state_space->setBounds(bounds);
    real_vector_state_space->setLongestValidSegmentFraction(
        config_.collision_checking.longest_valid_segment_fraction);
    state_space_ = real_vector_state_space;

    space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);
    collision_interface_ =
        std::make_shared<DualArmCollisionInterface>(config_, robot_model_, getPlanningScene(), state_space_);
    space_information_->setStateValidityChecker(
        [this](const ompl::base::State* state) { return collision_interface_->isStateValid(state); });
    double effective_state_validity_resolution = config_.collision_checking.state_validity_checking_resolution;
    const double maximum_extent = state_space_ ? state_space_->getMaximumExtent() : 0.0;
    if (maximum_extent > std::numeric_limits<double>::epsilon() && config_.solve.interpolation_step > 0.0)
    {
      effective_state_validity_resolution =
          std::min(effective_state_validity_resolution, config_.solve.interpolation_step / maximum_extent);
    }
    space_information_->setStateValidityCheckingResolution(effective_state_validity_resolution);
    space_information_->setup();
  }

  moveit::core::RobotState resolveStartState() const
  {
    moveit::core::RobotState start_robot_state(getPlanningScene()->getCurrentState());
    const sensor_msgs::JointState& start_joint_state = request_.start_state.joint_state;

    if (start_joint_state.name.size() != start_joint_state.position.size())
    {
      throw std::runtime_error("MotionPlanRequest.start_state.joint_state has mismatched name and position arrays.");
    }

    for (std::size_t index = 0; index < start_joint_state.name.size(); ++index)
    {
      start_robot_state.setVariablePosition(start_joint_state.name[index], start_joint_state.position[index]);
    }

    start_robot_state.update();
    return start_robot_state;
  }

  GoalRegionSpecification resolveGoalRegionSpecification(const std::vector<double>& reference_joint_values) const
  {
    if (request_.goal_constraints.empty())
    {
      throw std::runtime_error("MotionPlanRequest.goal_constraints is empty.");
    }

    if (!request_.path_constraints.joint_constraints.empty() || !request_.path_constraints.position_constraints.empty() ||
        !request_.path_constraints.orientation_constraints.empty() ||
        !request_.path_constraints.visibility_constraints.empty())
    {
      throw std::runtime_error("Path constraints are not supported by this plugin.");
    }

    const moveit_msgs::Constraints& goal_constraint_set = request_.goal_constraints.front();
    if (!goal_constraint_set.position_constraints.empty() || !goal_constraint_set.orientation_constraints.empty() ||
        !goal_constraint_set.visibility_constraints.empty())
    {
      throw std::runtime_error("Only joint-space goal constraints are supported by this plugin.");
    }

    if (goal_constraint_set.joint_constraints.empty())
    {
      throw std::runtime_error("Goal constraints do not contain any joint constraints.");
    }

    std::map<std::string, moveit_msgs::JointConstraint> constrained_joint_positions;
    for (const moveit_msgs::JointConstraint& joint_constraint : goal_constraint_set.joint_constraints)
    {
      if (joint_constraint.tolerance_above < 0.0 || joint_constraint.tolerance_below < 0.0)
      {
        throw std::runtime_error("Joint constraint tolerances must be non-negative for joint \"" +
                                 joint_constraint.joint_name + "\".");
      }

      if (constrained_joint_positions.count(joint_constraint.joint_name) > 0U)
      {
        throw std::runtime_error("Goal constraints contain duplicate entries for joint \"" +
                                 joint_constraint.joint_name + "\".");
      }

      constrained_joint_positions[joint_constraint.joint_name] = joint_constraint;
    }

    GoalRegionSpecification goal_region_specification;
    goal_region_specification.nominal_joint_values.reserve(config_.robot.joint_order.size());
    goal_region_specification.lower_joint_values.reserve(config_.robot.joint_order.size());
    goal_region_specification.upper_joint_values.reserve(config_.robot.joint_order.size());
    goal_region_specification.fixed_joint_mask.assign(config_.robot.joint_order.size(), false);
    goal_region_specification.fixed_joint_values = reference_joint_values;
    goal_region_specification.success_distance_threshold = config_.solve.goal_tolerance;

    const std::vector<std::string> left_arm_variable_names = left_arm_joint_model_group_->getVariableNames();
    const std::vector<std::string> right_arm_variable_names = right_arm_joint_model_group_->getVariableNames();
    const std::set<std::string> left_arm_joint_names(left_arm_variable_names.begin(), left_arm_variable_names.end());
    const std::set<std::string> right_arm_joint_names(right_arm_variable_names.begin(),
                                                      right_arm_variable_names.end());

    const auto isArmStatic = [&](const std::set<std::string>& arm_joint_names) {
      for (std::size_t index = 0; index < config_.robot.joint_order.size(); ++index)
      {
        const std::string& joint_name = config_.robot.joint_order[index];
        if (arm_joint_names.count(joint_name) == 0U)
        {
          continue;
        }

        const auto iterator = constrained_joint_positions.find(joint_name);
        if (iterator == constrained_joint_positions.end())
        {
          continue;
        }

        const JointBoundConfig& joint_bound = config_.state_space.joint_bounds.at(joint_name);
        const moveit_msgs::JointConstraint& joint_constraint = iterator->second;
        const double lower_goal_bound =
            std::max(joint_bound.lower, joint_constraint.position - joint_constraint.tolerance_below);
        const double upper_goal_bound =
            std::min(joint_bound.upper, joint_constraint.position + joint_constraint.tolerance_above);
        const double reference_value = reference_joint_values[index];

        if (reference_value < lower_goal_bound || reference_value > upper_goal_bound)
        {
          return false;
        }
      }

      return true;
    };

    goal_region_specification.left_arm_locked = isArmStatic(left_arm_joint_names);
    goal_region_specification.right_arm_locked = isArmStatic(right_arm_joint_names);

    for (std::size_t index = 0; index < config_.robot.joint_order.size(); ++index)
    {
      const std::string& joint_name = config_.robot.joint_order[index];
      const JointBoundConfig& joint_bound = config_.state_space.joint_bounds.at(joint_name);
      const auto iterator = constrained_joint_positions.find(joint_name);
      const bool joint_is_locked =
          (goal_region_specification.left_arm_locked && left_arm_joint_names.count(joint_name) > 0U) ||
          (goal_region_specification.right_arm_locked && right_arm_joint_names.count(joint_name) > 0U);

      if (joint_is_locked)
      {
        goal_region_specification.nominal_joint_values.push_back(reference_joint_values[index]);
        goal_region_specification.lower_joint_values.push_back(reference_joint_values[index]);
        goal_region_specification.upper_joint_values.push_back(reference_joint_values[index]);
        goal_region_specification.fixed_joint_mask[index] = true;
        continue;
      }

      if (iterator == constrained_joint_positions.end())
      {
        goal_region_specification.nominal_joint_values.push_back(reference_joint_values[index]);
        goal_region_specification.lower_joint_values.push_back(joint_bound.lower);
        goal_region_specification.upper_joint_values.push_back(joint_bound.upper);
        continue;
      }

      const moveit_msgs::JointConstraint& joint_constraint = iterator->second;
      const double lower_goal_bound = std::max(joint_bound.lower, joint_constraint.position - joint_constraint.tolerance_below);
      const double upper_goal_bound = std::min(joint_bound.upper, joint_constraint.position + joint_constraint.tolerance_above);

      if (lower_goal_bound > upper_goal_bound)
      {
        throw std::runtime_error("Joint goal interval is empty after applying tolerances and joint bounds for joint \"" +
                                 joint_name + "\".");
      }

      goal_region_specification.nominal_joint_values.push_back(
          std::min(std::max(joint_constraint.position, lower_goal_bound), upper_goal_bound));
      goal_region_specification.lower_joint_values.push_back(lower_goal_bound);
      goal_region_specification.upper_joint_values.push_back(upper_goal_bound);
    }

    return goal_region_specification;
  }

  bool isStateWithinGoalRegion(const std::vector<double>& joint_values,
                               const GoalRegionSpecification& goal_region_specification) const
  {
    if (joint_values.size() != goal_region_specification.lower_joint_values.size() ||
        joint_values.size() != goal_region_specification.upper_joint_values.size())
    {
      return false;
    }

    return computeGoalDistance(joint_values, goal_region_specification) <=
           goal_region_specification.success_distance_threshold;
  }

  std::vector<double> extractJointValues(const moveit::core::RobotState& robot_state) const
  {
    std::vector<double> joint_values;
    joint_values.reserve(config_.robot.joint_order.size());

    for (const std::string& joint_name : config_.robot.joint_order)
    {
      joint_values.push_back(robot_state.getVariablePosition(joint_name));
    }

    return joint_values;
  }

  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> makeScopedState(
      const std::vector<double>& joint_values) const
  {
    if (joint_values.size() != config_.robot.configuration_dimension)
    {
      throw std::runtime_error("Joint value vector dimension does not match the configured state space dimension.");
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> scoped_state(state_space_);
    auto* state_values = scoped_state->as<ompl::base::RealVectorStateSpace::StateType>();
    for (std::size_t index = 0; index < joint_values.size(); ++index)
    {
      state_values->values[index] = joint_values[index];
    }
    return scoped_state;
  }

  ompl::base::PlannerPtr createPlanner(const ompl::base::ProblemDefinitionPtr& problem_definition,
                                       const ompl::base::State* goal_tree_seed_state) const
  {
    auto planner = std::make_shared<LocalTRRT>(space_information_);
    planner->setProblemDefinition(problem_definition);
    planner->setRange(config_.trrt.range);
    planner->setGoalBias(config_.trrt.goal_bias);
    planner->setEnableConnect(config_.trrt.enable_connect);
    planner->setSamplingStrategy(config_.trrt.sampling_strategy);
    planner->setGoalTreeSeedState(goal_tree_seed_state);
    planner->setSobolSplitIndex(static_cast<unsigned int>(left_arm_joint_model_group_->getVariableCount()));
    planner->setSobolCandidatePoolSize(static_cast<unsigned int>(config_.trrt.sobol_candidate_pool_size));
    planner->setSobolSortByGoalDistance(config_.trrt.sobol_sort_by_goal_distance);
    planner->setTempChangeFactor(config_.trrt.temp_change_factor);
    planner->setInitTemperature(config_.trrt.init_temperature);
    planner->setFrontierThreshold(config_.trrt.frontier_threshold);
    planner->setFrontierNodeRatio(config_.trrt.frontier_node_ratio);
    planner->setCostThreshold(config_.trrt.cost_threshold);
    planner->setMinParentChildDistance(config_.trrt.min_parent_child_distance);
    planner->setMinParentChildDistanceScale(config_.trrt.min_parent_child_distance_scale);
    planner->setMaxSamplingAttemptsPerIteration(
        static_cast<unsigned int>(config_.trrt.max_sampling_attempts_per_iteration));
    planner->setup();
    return planner;
  }

  std::unique_ptr<ompl::base::PlannerData> capturePlannerData(const ompl::base::PlannerPtr& planner) const
  {
    if ((!tree_visualizer_ && !planner_stats_publisher_) || !planner)
    {
      return nullptr;
    }

    auto planner_data = std::make_unique<ompl::base::PlannerData>(space_information_);
    planner->getPlannerData(*planner_data);
    planner_data->decoupleFromPlanner();
    return planner_data;
  }

  void publishPlannerStats(const ompl::base::PlannerData* planner_data, bool planning_succeeded, int attempt_index,
                           const ompl::geometric::PathGeometric* solution_path = nullptr) const
  {
    if (!planner_stats_publisher_ || !planner_data)
    {
      return;
    }

    std_msgs::String message;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "{"
           << "\"stamp\":" << ros::Time::now().toSec() << ","
           << "\"planner_name\":\"" << config_.trrt.planner_config_name << "\","
           << "\"planning_succeeded\":" << (planning_succeeded ? "true" : "false") << ","
           << "\"attempt_index\":" << attempt_index << ","
           << "\"tree_vertices\":" << planner_data->numVertices() << ","
           << "\"tree_edges\":" << planner_data->numEdges() << ","
           << "\"solution_states\":" << (solution_path ? solution_path->getStateCount() : 0U)
           << "}";
    message.data = stream.str();
    planner_stats_publisher_.publish(message);
  }

  ompl::base::OptimizationObjectivePtr createOptimizationObjective() const
  {
    ompl::base::OptimizationObjectivePtr base_objective;
    if (config_.cost_objective.type == "clearance")
    {
      base_objective =
          std::make_shared<DualArmClearanceObjective>(space_information_, collision_interface_, config_);
    }
    else
    {
      base_objective = std::make_shared<ompl::base::MechanicalWorkOptimizationObjective>(space_information_);
    }

    return base_objective;
  }

  robot_trajectory::RobotTrajectoryPtr buildRobotTrajectory(const ompl::geometric::PathGeometric& path,
                                                            const moveit::core::RobotState& reference_state) const
  {
    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());

    for (std::size_t state_index = 0; state_index < path.getStateCount(); ++state_index)
    {
      const auto* state = path.getState(state_index)->as<ompl::base::RealVectorStateSpace::StateType>();
      moveit::core::RobotState robot_state(reference_state);

      for (std::size_t joint_index = 0; joint_index < config_.robot.joint_order.size(); ++joint_index)
      {
        robot_state.setVariablePosition(config_.robot.joint_order[joint_index], state->values[joint_index]);
      }

      robot_state.update();
      trajectory->addSuffixWayPoint(robot_state, 0.0);
    }

    trajectory->unwind();
    if (trajectory->getWayPointCount() > 1U)
    {
      trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
      time_parameterization.computeTimeStamps(*trajectory, 1.0, 1.0);
    }

    return trajectory;
  }

  robot_trajectory::RobotTrajectoryPtr buildSingleStateTrajectory(
      const moveit::core::RobotState& robot_state) const
  {
    auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, getGroupName());
    trajectory->addSuffixWayPoint(robot_state, 0.0);
    return trajectory;
  }

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  const moveit::core::JointModelGroup* left_arm_joint_model_group_;
  const moveit::core::JointModelGroup* right_arm_joint_model_group_;
  PlannerConfig config_;
  ompl::base::StateSpacePtr state_space_;
  ompl::base::SpaceInformationPtr space_information_;
  DualArmCollisionInterfacePtr collision_interface_;
  std::shared_ptr<DualArmTreeVisualizer> tree_visualizer_;
  ros::Publisher planner_stats_publisher_;
  ros::NodeHandle node_handle_;
  HeatmapFieldPtr heatmap_field_;
  bool heatmap_ready_{ false };
  std::atomic_bool terminate_requested_;
};
}  // namespace

DualArmTRRTPlannerManager::DualArmTRRTPlannerManager() : private_node_handle_("~")
{
}

bool DualArmTRRTPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns)
{
  robot_model_ = model;
  parameter_namespace_ = ns;
  pipeline_node_handle_ = ros::NodeHandle(ns);

  ROS_INFO_STREAM("[DualArmTRRT] Initializing MoveIt plugin under namespace " << parameter_namespace_ << ".");
  return static_cast<bool>(robot_model_);
}

bool DualArmTRRTPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
{
  if (req.group_name.empty())
  {
    return false;
  }

  std::string configured_group_name;
  if (!pipeline_node_handle_.getParam("robot/planning_group", configured_group_name))
  {
    return false;
  }

  if (req.group_name != configured_group_name)
  {
    return false;
  }

  if (req.goal_constraints.empty())
  {
    return false;
  }

  return true;
}

void DualArmTRRTPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();

  if (!config_settings_.empty())
  {
    std::set<std::string> unique_names;
    for (const auto& config_entry : config_settings_)
    {
      unique_names.insert(config_entry.second.name);
    }
    algs.assign(unique_names.begin(), unique_names.end());
    return;
  }

  std::string default_planner_config;
  if (pipeline_node_handle_.getParam("default_planner_config", default_planner_config))
  {
    algs.push_back(default_planner_config);
  }
}

planning_interface::PlanningContextPtr DualArmTRRTPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (!canServiceRequest(req))
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  try
  {
    const std::string planner_config_name = resolvePlannerConfigName(req);
    const PlannerConfig config = loadPlannerConfig(pipeline_node_handle_, planner_config_name);
    const std::shared_ptr<DualArmTreeVisualizer> tree_visualizer = getOrCreateTreeVisualizer(config);
    const ros::Publisher planner_stats_publisher = getOrCreatePlannerStatsPublisher();

    if (config.robot.planning_group != req.group_name)
    {
      error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
      return planning_interface::PlanningContextPtr();
    }

    if (config.trrt.planner_type != "geometric::TRRT" && config.trrt.planner_type != "ompl::geometric::TRRT" &&
        config.trrt.planner_type != "dual_arm_rrt_planner::LocalTRRT")
    {
      ROS_WARN_STREAM("[DualArmTRRT] planner_configs/" << planner_config_name
                                                       << "/type is " << config.trrt.planner_type
                                                       << ", but expected geometric::TRRT, "
                                                          "ompl::geometric::TRRT, or "
                                                          "dual_arm_rrt_planner::LocalTRRT.");
    }

    planning_interface::PlanningContextPtr context = std::make_shared<DualArmTRRTPlanningContext>(
        planner_config_name, req.group_name, robot_model_, config, tree_visualizer, pipeline_node_handle_,
        planner_stats_publisher);
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return context;
  }
  catch (const std::exception& exception)
  {
    ROS_ERROR_STREAM("[DualArmTRRT] Failed to construct planning context: " << exception.what());
    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }
}

std::shared_ptr<DualArmTreeVisualizer> DualArmTRRTPlannerManager::getOrCreateTreeVisualizer(
    const PlannerConfig& config) const
{
  if (!config.visualization.enabled)
  {
    return nullptr;
  }

  const std::string cache_key = config.trrt.planner_config_name + "|" + config.visualization.marker_topic;
  const auto cached_visualizer = tree_visualizer_cache_.find(cache_key);
  if (cached_visualizer != tree_visualizer_cache_.end())
  {
    return cached_visualizer->second;
  }

  std::shared_ptr<DualArmTreeVisualizer> tree_visualizer =
      std::make_shared<DualArmTreeVisualizer>(config, robot_model_, pipeline_node_handle_);
  tree_visualizer_cache_[cache_key] = tree_visualizer;
  return tree_visualizer;
}

ros::Publisher DualArmTRRTPlannerManager::getOrCreatePlannerStatsPublisher() const
{
  if (!planner_stats_publisher_)
  {
    planner_stats_publisher_ =
        pipeline_node_handle_.advertise<std_msgs::String>("/dual_arm_trrt/planning_stats", 1, true);
  }
  return planner_stats_publisher_;
}

std::string DualArmTRRTPlannerManager::resolvePlannerConfigName(const planning_interface::MotionPlanRequest& req) const
{
  if (!req.planner_id.empty())
  {
    return req.planner_id;
  }

  std::string group_default_planner_config;
  if (pipeline_node_handle_.getParam(req.group_name + "/default_planner_config", group_default_planner_config))
  {
    return group_default_planner_config;
  }

  std::string global_default_planner_config;
  if (pipeline_node_handle_.getParam("default_planner_config", global_default_planner_config))
  {
    return global_default_planner_config;
  }

  throw std::runtime_error("No planner config name was provided in the request and no default_planner_config was "
                           "found in the pipeline namespace.");
}
}  // namespace dual_arm_rrt_planner

PLUGINLIB_EXPORT_CLASS(dual_arm_rrt_planner::DualArmTRRTPlannerManager, planning_interface::PlannerManager)

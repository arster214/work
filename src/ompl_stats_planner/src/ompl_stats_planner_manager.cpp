#include "ompl_stats_planner/ompl_stats_planner_manager.h"

#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>
#include <string>

#include <ros/console.h>
#include <std_msgs/String.h>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>

#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/planning_interface/planning_response.h>

#include <pluginlib/class_list_macros.hpp>

namespace ompl_stats_planner
{
namespace
{

std::string escapeJson(const std::string& input)
{
  std::ostringstream stream;
  for (const char ch : input)
  {
    switch (ch)
    {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        stream << ch;
        break;
    }
  }
  return stream.str();
}

std::string extractPlannerIdFromConfigKey(const std::string& config_key)
{
  const std::size_t open = config_key.find('[');
  const std::size_t close = config_key.find(']', open == std::string::npos ? 0U : open + 1U);
  if (open != std::string::npos && close != std::string::npos && close > open + 1U)
  {
    return config_key.substr(open + 1U, close - open - 1U);
  }
  return std::string();
}

double computeDetailedPlanningTime(const planning_interface::MotionPlanDetailedResponse& response)
{
  return std::accumulate(response.processing_time_.begin(), response.processing_time_.end(), 0.0);
}

class OMPLStatsPlanningContext : public planning_interface::PlanningContext
{
public:
  OMPLStatsPlanningContext(const ompl_interface::ModelBasedPlanningContextPtr& context,
                           const ros::Publisher& planner_stats_publisher)
    : planning_interface::PlanningContext(context ? context->getName() : "", context ? context->getGroupName() : "")
    , context_(context)
    , planner_stats_publisher_(planner_stats_publisher)
  {
    if (context_)
    {
      setPlanningScene(context_->getPlanningScene());
      setMotionPlanRequest(context_->getMotionPlanRequest());
    }
  }

  bool solve(planning_interface::MotionPlanResponse& res) override
  {
    if (!context_)
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    const bool success = context_->solve(res);
    publishPlannerStats(success, res.planner_id_, res.planning_time_);
    return success;
  }

  bool solve(planning_interface::MotionPlanDetailedResponse& res) override
  {
    if (!context_)
    {
      res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
      return false;
    }

    const bool success = context_->solve(res);
    publishPlannerStats(success, res.planner_id_, computeDetailedPlanningTime(res));
    return success;
  }

  void clear() override
  {
    if (context_)
    {
      context_->clear();
    }
  }

  bool terminate() override
  {
    return context_ ? context_->terminate() : false;
  }

private:
  void publishPlannerStats(bool planning_succeeded, const std::string& response_planner_id,
                           double response_planning_time) const
  {
    if (!context_ || !planner_stats_publisher_)
    {
      return;
    }

    const auto& simple_setup = context_->getOMPLSimpleSetup();
    if (!simple_setup)
    {
      return;
    }

    const ompl::base::PlannerPtr planner = simple_setup->getPlanner();
    if (!planner)
    {
      return;
    }

    ompl::base::PlannerData planner_data(simple_setup->getSpaceInformation());
    planner->getPlannerData(planner_data);

    std::size_t solution_states = 0U;
    if (simple_setup->haveSolutionPath())
    {
      solution_states = simple_setup->getSolutionPath().getStateCount();
    }

    const unsigned int requested_planning_attempts =
        getMotionPlanRequest().num_planning_attempts > 0 ? getMotionPlanRequest().num_planning_attempts : 1U;
    const bool planner_data_available = !(
        requested_planning_attempts > 1U && planner_data.numVertices() == 0U && planner_data.numEdges() == 0U &&
        solution_states > 0U);

    const std::string requested_planner_id = getMotionPlanRequest().planner_id;
    const std::string planner_id = !requested_planner_id.empty() ? requested_planner_id : response_planner_id;
    const double planning_time = response_planning_time > 0.0 ?
                                     response_planning_time :
                                     (context_->getLastPlanTime() + context_->getLastSimplifyTime());

    std_msgs::String message;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6)
           << "{"
           << "\"stamp\":" << ros::Time::now().toSec() << ","
           << "\"pipeline\":\"ompl\","
           << "\"group_name\":\"" << escapeJson(getGroupName()) << "\","
           << "\"planner_name\":\"" << escapeJson(planner->getName()) << "\","
           << "\"planner_id\":\"" << escapeJson(planner_id) << "\","
           << "\"requested_planning_attempts\":" << requested_planning_attempts << ","
           << "\"planner_data_available\":" << (planner_data_available ? "true" : "false") << ","
           << "\"planning_succeeded\":" << (planning_succeeded ? "true" : "false") << ","
           << "\"tree_vertices\":";
    if (planner_data_available)
      stream << planner_data.numVertices();
    else
      stream << "null";
    stream << ","
           << "\"tree_edges\":";
    if (planner_data_available)
      stream << planner_data.numEdges();
    else
      stream << "null";
    stream << ","
           << "\"solution_states\":" << solution_states << ","
           << "\"planning_time\":" << planning_time
           << "}";
    message.data = stream.str();
    planner_stats_publisher_.publish(message);
  }

  ompl_interface::ModelBasedPlanningContextPtr context_;
  ros::Publisher planner_stats_publisher_;
};

}  // namespace

OMPLStatsPlannerManager::OMPLStatsPlannerManager() = default;

bool OMPLStatsPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns)
{
  robot_model_ = model;
  nh_ = ros::NodeHandle(ns);

  if (!robot_model_)
  {
    ROS_ERROR("[OMPLStatsPlanner] Robot model is null during initialization.");
    return false;
  }

  nh_.param<std::string>("ompl_planning_stats_topic", planner_stats_topic_, "/ompl_planning_stats");
  planner_stats_publisher_ = nh_.advertise<std_msgs::String>(planner_stats_topic_, 1, true);

  ompl_interface_ = std::make_shared<ompl_interface::OMPLInterface>(robot_model_, nh_);
  if (!config_settings_.empty())
  {
    ompl_interface_->setPlannerConfigurations(config_settings_);
  }
  else
  {
    config_settings_ = ompl_interface_->getPlannerConfigurations();
  }

  ROS_INFO_STREAM("[OMPLStatsPlanner] Initialized. Publishing PlannerData stats to " << planner_stats_topic_);
  return true;
}

bool OMPLStatsPlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& /*req*/) const
{
  return static_cast<bool>(ompl_interface_);
}

std::string OMPLStatsPlannerManager::getDescription() const
{
  return "OMPL with PlannerData Stats";
}

void OMPLStatsPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();

  std::set<std::string> unique_algorithms;
  for (const auto& entry : config_settings_)
  {
    const std::string planner_id = extractPlannerIdFromConfigKey(entry.first);
    if (!planner_id.empty())
    {
      unique_algorithms.insert(planner_id);
    }
  }

  algs.assign(unique_algorithms.begin(), unique_algorithms.end());
}

planning_interface::PlanningContextPtr OMPLStatsPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (!ompl_interface_)
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }

  ompl_interface::ModelBasedPlanningContextPtr context =
      ompl_interface_->getPlanningContext(planning_scene, req, error_code);
  if (!context)
  {
    return planning_interface::PlanningContextPtr();
  }

  return planning_interface::PlanningContextPtr(
      new OMPLStatsPlanningContext(context, planner_stats_publisher_));
}

void OMPLStatsPlannerManager::setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs)
{
  planning_interface::PlannerManager::setPlannerConfigurations(pcs);
  if (ompl_interface_)
  {
    ompl_interface_->setPlannerConfigurations(pcs);
  }
}

}  // namespace ompl_stats_planner

PLUGINLIB_EXPORT_CLASS(ompl_stats_planner::OMPLStatsPlannerManager, planning_interface::PlannerManager)

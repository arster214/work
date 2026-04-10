#ifndef OMPL_STATS_PLANNER_OMPL_STATS_PLANNER_MANAGER_H
#define OMPL_STATS_PLANNER_OMPL_STATS_PLANNER_MANAGER_H

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

namespace ompl_stats_planner
{

class OMPLStatsPlannerManager : public planning_interface::PlannerManager
{
public:
  OMPLStatsPlannerManager();
  ~OMPLStatsPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override;
  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;
  std::string getDescription() const override;
  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene,
      const planning_interface::MotionPlanRequest& req,
      moveit_msgs::MoveItErrorCodes& error_code) const override;
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pcs) override;

private:
  ros::NodeHandle nh_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::shared_ptr<ompl_interface::OMPLInterface> ompl_interface_;
  ros::Publisher planner_stats_publisher_;
  std::string planner_stats_topic_;
};

}  // namespace ompl_stats_planner

#endif  // OMPL_STATS_PLANNER_OMPL_STATS_PLANNER_MANAGER_H

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <ros/ros.h>

namespace dual_arm_rrt_planner
{
class DualArmTreeVisualizer;
struct PlannerConfig;

class DualArmTRRTPlannerManager : public planning_interface::PlannerManager
{
public:
  DualArmTRRTPlannerManager();

  ~DualArmTRRTPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;

  std::string getDescription() const override
  {
    return "Dual Arm Native TRRT";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req,
      moveit_msgs::MoveItErrorCodes& error_code) const override;

private:
  std::shared_ptr<DualArmTreeVisualizer> getOrCreateTreeVisualizer(const PlannerConfig& config) const;
  ros::Publisher getOrCreatePlannerStatsPublisher() const;

  std::string resolvePlannerConfigName(const planning_interface::MotionPlanRequest& req) const;

  ros::NodeHandle private_node_handle_;
  mutable ros::NodeHandle pipeline_node_handle_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string parameter_namespace_;
  mutable std::map<std::string, std::shared_ptr<DualArmTreeVisualizer> > tree_visualizer_cache_;
  mutable ros::Publisher planner_stats_publisher_;
};
}  // namespace dual_arm_rrt_planner

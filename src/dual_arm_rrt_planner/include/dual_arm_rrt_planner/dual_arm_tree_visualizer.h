#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/PlannerData.h>
#include <ompl/geometric/PathGeometric.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <dual_arm_rrt_planner/planner_config.h>

namespace dual_arm_rrt_planner
{
class DualArmTreeVisualizer
{
public:
  DualArmTreeVisualizer(const PlannerConfig& config, const moveit::core::RobotModelConstPtr& robot_model,
                        ros::NodeHandle node_handle);

  void clear();

  void publishPlannerData(const ompl::base::PlannerData& planner_data, const std::string& planner_name,
                          bool planning_succeeded, int attempt_index,
                          const ompl::geometric::PathGeometric* solution_branch_path = nullptr,
                          const ompl::geometric::PathGeometric* solution_path = nullptr,
                          bool left_arm_active = true, bool right_arm_active = true);

  bool publishSolutionPathOnly(const std::string& planner_name, const ompl::geometric::PathGeometric& solution_path,
                               bool left_arm_active = true, bool right_arm_active = true);

private:
  struct EndEffectorPointPair
  {
    geometry_msgs::Point left_point;
    geometry_msgs::Point right_point;
    bool valid{ false };
  };

  EndEffectorPointPair computeEndEffectorPoints(const ompl::base::State* state) const;

  void collectPathPoints(const ompl::geometric::PathGeometric& path, std::vector<geometry_msgs::Point>& left_points,
                         std::vector<geometry_msgs::Point>& right_points) const;

  moveit::core::RobotState toRobotState(const ompl::base::State* state) const;

  visualization_msgs::Marker makeDeleteAllMarker(const ros::Time& stamp) const;

  visualization_msgs::Marker makeLineListMarker(const std::string& marker_namespace, int marker_id,
                                                const ros::Time& stamp, const ColorConfig& color,
                                                const std::vector<geometry_msgs::Point>& points) const;

  visualization_msgs::Marker makeLineStripMarker(const std::string& marker_namespace, int marker_id,
                                                 const ros::Time& stamp, const ColorConfig& color, double scale,
                                                 const std::vector<geometry_msgs::Point>& points) const;

  visualization_msgs::Marker makeSphereListMarker(const std::string& marker_namespace, int marker_id,
                                                  const ros::Time& stamp, const ColorConfig& color,
                                                  const std::vector<geometry_msgs::Point>& points) const;

  PlannerConfig config_;
  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::LinkModel* left_tip_link_model_{ nullptr };
  const moveit::core::LinkModel* right_tip_link_model_{ nullptr };
  ros::Publisher marker_publisher_;
};
}  // namespace dual_arm_rrt_planner

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <ros/ros.h>

#include <clearance_heatmap_msgs/EnvironmentInfo.h>
#include <clearance_heatmap_msgs/GenerateClearanceHeatmap.h>

namespace dual_arm_rrt_planner
{
class HeatmapField
{
public:
  struct GridMetadata
  {
    double resolution{ 0.0 };
    Eigen::Vector3d min_bound{ Eigen::Vector3d::Zero() };
    Eigen::Vector3d max_bound{ Eigen::Vector3d::Zero() };
    int dim_x{ 0 };
    int dim_y{ 0 };
    int dim_z{ 0 };
  };

  HeatmapField(const ros::NodeHandle& node_handle, const std::string& environment_topic = "/map_process/environment_info",
               const std::string& service_name = "/clearance_heatmap_server/generate_clearance_heatmap");

  bool hasEnvironmentInfo() const;

  bool ready() const;

  void setEnvironmentInfo(const clearance_heatmap_msgs::EnvironmentInfo& env_info);

  clearance_heatmap_msgs::EnvironmentInfo latestEnvironmentInfo() const;

  bool requestField(double resolution, const Eigen::Vector3d& min_bound, const Eigen::Vector3d& max_bound,
                    double wait_timeout_seconds = 0.0, std::string* error_message = nullptr);

  double valueAt(const Eigen::Vector3d& point) const;

  Eigen::Vector3d gradientAt(const Eigen::Vector3d& point, double delta = -1.0) const;

  GridMetadata grid() const;

private:
  void environmentCallback(const clearance_heatmap_msgs::EnvironmentInfoConstPtr& message);

  double sampleAtIndexLocked(int x_idx, int y_idx, int z_idx) const;

  ros::NodeHandle node_handle_;
  ros::Subscriber environment_subscriber_;
  ros::ServiceClient heatmap_client_;
  std::string environment_topic_;
  std::string service_name_;

  mutable std::mutex environment_mutex_;
  clearance_heatmap_msgs::EnvironmentInfo latest_environment_info_;
  bool has_environment_info_{ false };

  mutable std::mutex grid_mutex_;
  GridMetadata grid_metadata_;
  std::vector<float> heatmap_data_;
  bool grid_ready_{ false };
};

using HeatmapFieldPtr = std::shared_ptr<HeatmapField>;
}  // namespace dual_arm_rrt_planner

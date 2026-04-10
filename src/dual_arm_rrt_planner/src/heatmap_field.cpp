#include <dual_arm_rrt_planner/heatmap_field.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <ros/topic.h>

namespace dual_arm_rrt_planner
{
namespace
{
double clampScalar(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

std::size_t flattenIndex(const int x_idx, const int y_idx, const int z_idx, const HeatmapField::GridMetadata& metadata)
{
  return static_cast<std::size_t>(z_idx * metadata.dim_x * metadata.dim_y + y_idx * metadata.dim_x + x_idx);
}
}  // namespace

HeatmapField::HeatmapField(const ros::NodeHandle& node_handle, const std::string& environment_topic,
                           const std::string& service_name)
  : node_handle_(node_handle), environment_topic_(environment_topic), service_name_(service_name)
{
  environment_subscriber_ =
      node_handle_.subscribe(environment_topic_, 1, &HeatmapField::environmentCallback, this);
  heatmap_client_ = node_handle_.serviceClient<clearance_heatmap_msgs::GenerateClearanceHeatmap>(service_name_, true);
}

bool HeatmapField::hasEnvironmentInfo() const
{
  std::lock_guard<std::mutex> environment_lock(environment_mutex_);
  return has_environment_info_;
}

bool HeatmapField::ready() const
{
  std::lock_guard<std::mutex> grid_lock(grid_mutex_);
  return grid_ready_;
}

void HeatmapField::setEnvironmentInfo(const clearance_heatmap_msgs::EnvironmentInfo& env_info)
{
  std::lock_guard<std::mutex> environment_lock(environment_mutex_);
  latest_environment_info_ = env_info;
  has_environment_info_ = true;
}

clearance_heatmap_msgs::EnvironmentInfo HeatmapField::latestEnvironmentInfo() const
{
  std::lock_guard<std::mutex> environment_lock(environment_mutex_);
  return latest_environment_info_;
}

bool HeatmapField::requestField(double resolution, const Eigen::Vector3d& min_bound, const Eigen::Vector3d& max_bound,
                                double wait_timeout_seconds, std::string* error_message)
{
  if (resolution <= 0.0)
  {
    if (error_message)
    {
      *error_message = "HeatmapField requires a positive grid resolution.";
    }
    return false;
  }

  if (!hasEnvironmentInfo() && wait_timeout_seconds > 0.0)
  {
    const auto message = ros::topic::waitForMessage<clearance_heatmap_msgs::EnvironmentInfo>(
        environment_topic_, node_handle_, ros::Duration(wait_timeout_seconds));
    if (message)
    {
      setEnvironmentInfo(*message);
    }
  }

  clearance_heatmap_msgs::EnvironmentInfo env_info;
  {
    std::lock_guard<std::mutex> environment_lock(environment_mutex_);
    if (!has_environment_info_)
    {
      if (error_message)
      {
        *error_message = "No EnvironmentInfo has been received yet.";
      }
      return false;
    }
    env_info = latest_environment_info_;
  }

  if (wait_timeout_seconds > 0.0 &&
      !heatmap_client_.waitForExistence(ros::Duration(wait_timeout_seconds)))
  {
    if (error_message)
    {
      *error_message = "Timed out waiting for the GenerateClearanceHeatmap service.";
    }
    return false;
  }

  clearance_heatmap_msgs::GenerateClearanceHeatmap service_call;
  service_call.request.env_info = env_info;
  service_call.request.resolution = static_cast<float>(resolution);
  service_call.request.min_bound.x = min_bound.x();
  service_call.request.min_bound.y = min_bound.y();
  service_call.request.min_bound.z = min_bound.z();
  service_call.request.max_bound.x = max_bound.x();
  service_call.request.max_bound.y = max_bound.y();
  service_call.request.max_bound.z = max_bound.z();

  if (!heatmap_client_.call(service_call))
  {
    if (error_message)
    {
      *error_message = "Failed to call the GenerateClearanceHeatmap service.";
    }
    return false;
  }

  const auto& response = service_call.response;
  if (!response.success)
  {
    if (error_message)
    {
      *error_message = response.message;
    }
    return false;
  }

  if (response.dim_x <= 0 || response.dim_y <= 0 || response.dim_z <= 0)
  {
    if (error_message)
    {
      *error_message = "GenerateClearanceHeatmap returned invalid grid dimensions.";
    }
    return false;
  }

  const std::size_t expected_size =
      static_cast<std::size_t>(response.dim_x) * static_cast<std::size_t>(response.dim_y) *
      static_cast<std::size_t>(response.dim_z);
  if (response.heatmap_data.size() != expected_size)
  {
    if (error_message)
    {
      *error_message = "GenerateClearanceHeatmap returned a data buffer whose size does not match the reported dimensions.";
    }
    return false;
  }

  GridMetadata metadata;
  metadata.resolution = resolution;
  metadata.min_bound = min_bound;
  metadata.dim_x = response.dim_x;
  metadata.dim_y = response.dim_y;
  metadata.dim_z = response.dim_z;
  metadata.max_bound = metadata.min_bound +
                       resolution * Eigen::Vector3d(static_cast<double>(metadata.dim_x - 1),
                                                    static_cast<double>(metadata.dim_y - 1),
                                                    static_cast<double>(metadata.dim_z - 1));

  {
    std::lock_guard<std::mutex> grid_lock(grid_mutex_);
    grid_metadata_ = metadata;
    heatmap_data_ = response.heatmap_data;
    grid_ready_ = true;
  }

  return true;
}

double HeatmapField::valueAt(const Eigen::Vector3d& point) const
{
  std::lock_guard<std::mutex> grid_lock(grid_mutex_);
  if (!grid_ready_ || heatmap_data_.empty() || grid_metadata_.resolution <= 0.0)
  {
    return 0.0;
  }

  if (!point.allFinite())
  {
    return 0.0;
  }

  const int max_x_index = std::max(0, grid_metadata_.dim_x - 1);
  const int max_y_index = std::max(0, grid_metadata_.dim_y - 1);
  const int max_z_index = std::max(0, grid_metadata_.dim_z - 1);

  const double fx =
      grid_metadata_.dim_x > 1
          ? clampScalar((point.x() - grid_metadata_.min_bound.x()) / grid_metadata_.resolution, 0.0,
                        static_cast<double>(max_x_index))
          : 0.0;
  const double fy =
      grid_metadata_.dim_y > 1
          ? clampScalar((point.y() - grid_metadata_.min_bound.y()) / grid_metadata_.resolution, 0.0,
                        static_cast<double>(max_y_index))
          : 0.0;
  const double fz =
      grid_metadata_.dim_z > 1
          ? clampScalar((point.z() - grid_metadata_.min_bound.z()) / grid_metadata_.resolution, 0.0,
                        static_cast<double>(max_z_index))
          : 0.0;

  const int x0 = static_cast<int>(std::floor(fx));
  const int y0 = static_cast<int>(std::floor(fy));
  const int z0 = static_cast<int>(std::floor(fz));
  const int x1 = std::min(x0 + 1, max_x_index);
  const int y1 = std::min(y0 + 1, max_y_index);
  const int z1 = std::min(z0 + 1, max_z_index);

  const double tx = fx - static_cast<double>(x0);
  const double ty = fy - static_cast<double>(y0);
  const double tz = fz - static_cast<double>(z0);

  const double c000 = sampleAtIndexLocked(x0, y0, z0);
  const double c100 = sampleAtIndexLocked(x1, y0, z0);
  const double c010 = sampleAtIndexLocked(x0, y1, z0);
  const double c110 = sampleAtIndexLocked(x1, y1, z0);
  const double c001 = sampleAtIndexLocked(x0, y0, z1);
  const double c101 = sampleAtIndexLocked(x1, y0, z1);
  const double c011 = sampleAtIndexLocked(x0, y1, z1);
  const double c111 = sampleAtIndexLocked(x1, y1, z1);

  const double c00 = c000 * (1.0 - tx) + c100 * tx;
  const double c10 = c010 * (1.0 - tx) + c110 * tx;
  const double c01 = c001 * (1.0 - tx) + c101 * tx;
  const double c11 = c011 * (1.0 - tx) + c111 * tx;
  const double c0 = c00 * (1.0 - ty) + c10 * ty;
  const double c1 = c01 * (1.0 - ty) + c11 * ty;
  return c0 * (1.0 - tz) + c1 * tz;
}

Eigen::Vector3d HeatmapField::gradientAt(const Eigen::Vector3d& point, double delta) const
{
  const GridMetadata metadata = grid();
  if (metadata.resolution <= 0.0)
  {
    return Eigen::Vector3d::Zero();
  }

  const double step = delta > 0.0 ? delta : metadata.resolution;
  const Eigen::Vector3d dx(step, 0.0, 0.0);
  const Eigen::Vector3d dy(0.0, step, 0.0);
  const Eigen::Vector3d dz(0.0, 0.0, step);

  return Eigen::Vector3d((valueAt(point + dx) - valueAt(point - dx)) / (2.0 * step),
                         (valueAt(point + dy) - valueAt(point - dy)) / (2.0 * step),
                         (valueAt(point + dz) - valueAt(point - dz)) / (2.0 * step));
}

HeatmapField::GridMetadata HeatmapField::grid() const
{
  std::lock_guard<std::mutex> grid_lock(grid_mutex_);
  return grid_metadata_;
}

void HeatmapField::environmentCallback(const clearance_heatmap_msgs::EnvironmentInfoConstPtr& message)
{
  if (!message)
  {
    return;
  }

  std::lock_guard<std::mutex> environment_lock(environment_mutex_);
  latest_environment_info_ = *message;
  has_environment_info_ = true;
}

double HeatmapField::sampleAtIndexLocked(int x_idx, int y_idx, int z_idx) const
{
  x_idx = std::max(0, std::min(x_idx, grid_metadata_.dim_x - 1));
  y_idx = std::max(0, std::min(y_idx, grid_metadata_.dim_y - 1));
  z_idx = std::max(0, std::min(z_idx, grid_metadata_.dim_z - 1));

  const std::size_t index = flattenIndex(x_idx, y_idx, z_idx, grid_metadata_);
  if (index >= heatmap_data_.size())
  {
    return 0.0;
  }
  return static_cast<double>(heatmap_data_[index]);
}
}  // namespace dual_arm_rrt_planner

#ifndef DUAL_ARM_RRT_PLANNER_APF_MODULE_H
#define DUAL_ARM_RRT_PLANNER_APF_MODULE_H

#include "dual_arm_rrt_planner/modules/module_base.h"
#include <Eigen/Dense>

namespace dual_arm_planner
{
namespace modules
{

class APFModule : public ModuleBase
{
public:
    APFModule();
    virtual ~APFModule() = default;
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    void postExtend(PlanningContext& context) override;
    std::string getName() const override { return "APFModule"; }
    
    // 计算单臂受力
    Eigen::Vector3d computeTaskSpaceForce(const PlanningContext& context) const;
    Eigen::Vector3d computeTaskSpaceForce(const Eigen::Vector3d& current_pos, const Eigen::Vector3d& target_pos, const ObstacleInfo& obs_info) const;
    
    // 基础力计算函数
    Eigen::Vector3d calculateAttractiveForce(const Eigen::Vector3d& current, const Eigen::Vector3d& target) const;
    Eigen::Vector3d calculateRepulsiveForce(const ObstacleInfo& obs_info) const;

    // 将 force 作用到任务空间点上并返回修正后的任务点（APF 不应直接修改关节状态）
    Eigen::Vector3d applyForce(const Eigen::Vector3d& current_task_point, const Eigen::Vector3d& force, double step_size) const;
    // 计算点到 AABB 的距离与梯度（对外公开以便复用）
    std::pair<double, Eigen::Vector3d> distanceAndGradientToAABB(const Eigen::Vector3d& point, const AABB& box) const;

private:
    double attractive_gain_;
    double repulsive_gain_;
    double obstacle_influence_distance_;
    double decay_rate_;
    double max_force_magnitude_;
    double repulsive_force_threshold_;
    double near_field_distance_;
    double near_field_exponential_gain_;
};

} // namespace modules
} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_APF_MODULE_H

#ifndef IMPROVED_RRT_PLANNER_APF_MODULE_H
#define IMPROVED_RRT_PLANNER_APF_MODULE_H

#include "improved_rrt_planner/modules/module_base.h"
#include <Eigen/Dense>

namespace improved_rrt_planner
{
namespace modules
{

/**
 * @brief 人工势场（Artificial Potential Field）模块
 * 
 * 对新生成的节点施加势场力：
 * - 目标吸引力：将节点拉向目标
 * - 障碍物排斥力：将节点推离障碍物
 * 
 * 这有助于规划器在复杂环境中更智能地导航。
 */
class APFModule : public ModuleBase
{
public:
    APFModule();
    virtual ~APFModule() = default;
    
    /**
     * @brief 初始化模块
     * @param nh ROS节点句柄
     * @param name 模块名称
     * @return 初始化成功返回true
     */
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    
    /**
     * @brief 扩展后处理 - 对新节点施加势场力
     * @param context 规划上下文
     */
    void postExtend(PlanningContext& context) override;
    
    /**
     * @brief 获取模块名称
     * @return 模块名称
     */
    std::string getName() const override { return "APFModule"; }
    
private:
    /**
     * @brief 计算目标吸引力
     * @param current 当前状态
     * @param goal 目标状态
     * @return 吸引力向量
     */
    Eigen::VectorXd calculateAttractiveForce(const State& current, const State& goal) const;
    
    /**
     * @brief 计算障碍物排斥力
     * @param current 当前状态
     * @param obstacles 障碍物信息
     * @return 排斥力向量
     */
    Eigen::VectorXd calculateRepulsiveForce(const State& current, 
                                            const ObstacleInfo& obstacles) const;
    
    /**
     * @brief 对状态施加力
     * @param state 要调整的状态
     * @param force 力向量
     * @param step_size 步长（用于缩放力的影响）
     */
    void applyForce(State& state, const Eigen::VectorXd& force, double step_size) const;
    
    /**
     * @brief 使用数值梯度计算排斥力方向
     * @param current 当前状态
     * @param current_dist 当前到障碍物的距离
     * @return 梯度向量（指向距离增加的方向）
     */
    Eigen::VectorXd calculateNumericalGradient(const State& current, 
                                                double current_dist) const;
    
    /**
     * @brief 获取指定状态到最近障碍物的距离
     * @param state 要检查的状态
     * @return 到最近障碍物的距离
     */
    double getMinObstacleDistance(const State& state) const;
    
    // 基本参数
    double attractive_gain_;                // 吸引力增益 K_att
    double repulsive_gain_;                 // 排斥力增益 η (eta)
    double obstacle_influence_distance_;    // 障碍物影响距离 ρ₀ (rho_0)
    double decay_rate_;                     // 指数衰减率 σ (sigma)
    double max_force_magnitude_;            // 最大力限制（防止不稳定）
    
    // 排斥力配置
    bool use_repulsive_force_;              // 是否启用排斥力
    bool use_numerical_gradient_;           // 是否使用数值梯度计算方向
    double gradient_delta_;                 // 数值梯度的扰动量（弧度）
    
    // 性能优化
    double repulsive_force_threshold_;      // 只在距离小于此值时计算排斥力
    
    // 外部依赖（需要从PlanningContext获取）
    ros::NodeHandle nh_;
    std::string name_;
};

} // namespace modules
} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_APF_MODULE_H

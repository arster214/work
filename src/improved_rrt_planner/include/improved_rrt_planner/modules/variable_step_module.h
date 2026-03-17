#ifndef IMPROVED_RRT_PLANNER_VARIABLE_STEP_MODULE_H
#define IMPROVED_RRT_PLANNER_VARIABLE_STEP_MODULE_H

#include "improved_rrt_planner/modules/module_base.h"

namespace improved_rrt_planner
{
namespace modules
{

/**
 * @brief 可变步长模块
 * 
 * 根据障碍物密度和距离动态调整扩展步长。
 * 在开阔区域使用大步长快速探索，在障碍物密集区域使用小步长精细导航。
 */
class VariableStepModule : public ModuleBase
{
public:
    VariableStepModule();
    virtual ~VariableStepModule() = default;
    
    /**
     * @brief 初始化模块
     * @param nh ROS节点句柄
     * @param name 模块名称
     * @return 初始化成功返回true
     */
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    
    /**
     * @brief 扩展前处理 - 根据环境调整步长
     * @param context 规划上下文
     */
    void preExtend(PlanningContext& context) override;
    
    /**
     * @brief 获取模块名称
     * @return 模块名称
     */
    std::string getName() const override { return "VariableStepModule"; }
    
private:
    /**
     * @brief 计算步长
     * @param obstacles 障碍物信息
     * @return 计算出的步长
     */
    double calculateStepSize(const ObstacleInfo& obstacles) const;
    
    // 参数
    double min_step_size_;          // 最小步长
    double max_step_size_;          // 最大步长
    double density_weight_;         // 障碍物密度权重
    double distance_weight_;        // 障碍物距离权重
};

} // namespace modules
} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_VARIABLE_STEP_MODULE_H

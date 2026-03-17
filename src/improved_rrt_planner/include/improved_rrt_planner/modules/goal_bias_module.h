#ifndef IMPROVED_RRT_PLANNER_GOAL_BIAS_MODULE_H
#define IMPROVED_RRT_PLANNER_GOAL_BIAS_MODULE_H

#include "improved_rrt_planner/modules/module_base.h"
#include <random>

namespace improved_rrt_planner
{
namespace modules
{

/**
 * @brief 目标偏置模块
 * 
 * 以一定概率返回目标状态作为采样点，而不是随机采样。
 * 这可以加速规划器向目标收敛。
 */
class GoalBiasModule : public ModuleBase
{
public:
    GoalBiasModule();
    virtual ~GoalBiasModule() = default;
    
    /**
     * @brief 初始化模块
     * @param nh ROS节点句柄
     * @param name 模块名称
     * @return 初始化成功返回true
     */
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    
    /**
     * @brief 采样前处理 - 以goal_bias概率返回目标状态
     * @param context 规划上下文
     * @return 如果返回目标状态则返回true，否则返回false
     */
    bool preSample(PlanningContext& context) override;
    
    /**
     * @brief 获取模块名称
     * @return 模块名称
     */
    std::string getName() const override { return "GoalBiasModule"; }
    
private:
    double goal_bias_;                              // 目标偏置概率 [0, 1]
    std::mt19937 rng_;                              // 随机数生成器
    std::uniform_real_distribution<double> dist_;   // 均匀分布 [0, 1]
};

} // namespace modules
} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_GOAL_BIAS_MODULE_H

#ifndef IMPROVED_RRT_PLANNER_T_RRT_MODULE_H
#define IMPROVED_RRT_PLANNER_T_RRT_MODULE_H

#include "improved_rrt_planner/modules/module_base.h"

namespace improved_rrt_planner
{
namespace modules
{

/**
 * @brief T-RRT (Transition-based RRT) 模块
 * 
 * 使用基于温度的转换测试来过滤高代价状态，
 * 类似于模拟退火算法，帮助规划器避免高代价区域。
 */
class TRRTModule : public ModuleBase
{
public:
    TRRTModule();
    virtual ~TRRTModule() = default;
    
    /**
     * @brief 初始化模块
     * @param nh ROS节点句柄
     * @param name 模块名称
     * @return 初始化成功返回true
     */
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    
    /**
     * @brief 扩展后处理 - 执行转换测试
     * @param context 规划上下文
     * 
     * 如果转换测试失败，将 context.is_valid 设置为 false
     */
    void postExtend(PlanningContext& context) override;
    
    /**
     * @brief 清理 - 重置温度
     */
    void cleanup() override;
    
    /**
     * @brief 获取模块名称
     * @return 模块名称
     */
    std::string getName() const override { return "TRRTModule"; }
    
private:
    /**
     * @brief 转换测试 - 基于温度的代价过滤
     * @param motion_cost 运动代价
     * @return 接受返回true，拒绝返回false
     */
    bool transitionTest(double motion_cost);
    
    // T-RRT 参数
    double temp_;                    // 当前温度
    double init_temperature_;        // 初始温度
    double temp_change_factor_;      // 温度变化因子
    double worst_cost_;              // 树中最差代价
    double best_cost_;               // 树中最优代价
    double cost_threshold_;          // 代价阈值
};

} // namespace modules
} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_T_RRT_MODULE_H

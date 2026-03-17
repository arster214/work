#ifndef IMPROVED_RRT_PLANNER_NODE_REJECTION_MODULE_H
#define IMPROVED_RRT_PLANNER_NODE_REJECTION_MODULE_H

#include "improved_rrt_planner/modules/module_base.h"

namespace improved_rrt_planner
{
namespace modules
{

/**
 * @brief 节点拒绝模块
 * 
 * 拒绝与现有树节点过于接近的新节点，保持树的稀疏性。
 * 这可以减少冗余节点，提高规划效率。
 */
class NodeRejectionModule : public ModuleBase
{
public:
    NodeRejectionModule();
    virtual ~NodeRejectionModule() = default;
    
    /**
     * @brief 初始化模块
     * @param nh ROS节点句柄
     * @param name 模块名称
     * @return 初始化成功返回true
     */
    bool initialize(const ros::NodeHandle& nh, const std::string& name) override;
    
    /**
     * @brief 扩展后处理 - 检查新节点是否应该被拒绝
     * @param context 规划上下文
     */
    void postExtend(PlanningContext& context) override;
    
    /**
     * @brief 获取模块名称
     * @return 模块名称
     */
    std::string getName() const override { return "NodeRejectionModule"; }
    
private:
    /**
     * @brief 检查是否有节点在拒绝半径内
     * @param candidate 候选节点
     * @param radius 拒绝半径
     * @param tree 树
     * @return 如果有节点在半径内返回true
     */
    bool hasNearbyNodes(const State& candidate, double radius, 
                       const std::vector<Motion>& tree) const;
    
    /**
     * @brief 计算两个状态之间的距离
     * @param a 状态A
     * @param b 状态B
     * @return 距离
     */
    double distance(const State& a, const State& b) const;
    
    // 参数
    double rejection_radius_factor_;  // 拒绝半径因子（相对于步长）
};

} // namespace modules
} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_NODE_REJECTION_MODULE_H

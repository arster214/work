#ifndef BASELINE_RRT_BASIC_RRT_STAR_H
#define BASELINE_RRT_BASIC_RRT_STAR_H

#include "baseline_rrt/basic_rrt.h"

namespace baseline_rrt
{

// RRT* 的 Motion 需要额外的 cost 字段
struct MotionStar : public Motion
{
    double cost;  // 从起点到该节点的路径代价
    
    MotionStar() : Motion(), cost(0.0) {}
    explicit MotionStar(const State& s) : Motion(s), cost(0.0) {}
};

/**
 * @brief 基础 RRT* 实现
 * 
 * 特点：
 * - 继承自 BasicRRT
 * - 使用 KD-Tree 进行近邻搜索
 * - 实现 RRT* 的 rewiring 逻辑
 * - 无目标偏置
 * - 无其他优化
 */
class BasicRRTStar : public BasicRRT
{
public:
    BasicRRTStar();
    ~BasicRRTStar() override;
    
    // 设置 rewire 半径
    void setRewireRadius(double radius) { rewire_radius_ = radius; }
    
    // 重写 solve 函数以实现 RRT* 逻辑
    bool solve(const State& start, const State& goal, 
              std::vector<State>& path, double timeout) override;
    
    // 重写 getNodeCount（不再归一化，所有算法步长统一为0.3）
    size_t getNodeCount() const override { 
        return motions_.size(); 
    }
    
private:
    // RRT* 特有函数
    double pathCost(Motion* motion) const;
    std::vector<Motion*> getNearbyMotions(Motion* motion, double radius);
    void rewireTree(Motion* new_motion, const std::vector<Motion*>& neighbors);
    
    // RRT* 参数
    double rewire_radius_;
};

} // namespace baseline_rrt

#endif // BASELINE_RRT_BASIC_RRT_STAR_H

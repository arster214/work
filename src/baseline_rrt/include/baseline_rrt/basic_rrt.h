#ifndef BASELINE_RRT_BASIC_RRT_H
#define BASELINE_RRT_BASIC_RRT_H

#include <vector>
#include <memory>
#include <random>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

namespace baseline_rrt
{

// 状态表示
struct State
{
    std::vector<double> values;
    
    State() = default;
    explicit State(size_t dim) : values(dim, 0.0) {}
    State(const std::vector<double>& v) : values(v) {}
};

// Motion 节点
struct Motion
{
    State state;
    Motion* parent;
    
    Motion() : parent(nullptr) {}
    explicit Motion(const State& s) : state(s), parent(nullptr) {}
};

/**
 * @brief 基础 RRT 实现
 * 
 * 特点：
 * - 使用 KD-Tree (OMPL GNAT) 进行近邻搜索
 * - 无目标偏置（纯随机采样）
 * - 无其他优化
 */
class BasicRRT
{
public:
    BasicRRT();
    virtual ~BasicRRT();
    
    // 参数设置
    void setRange(double range) { max_distance_ = range; }
    void setStateDimension(size_t dim) { state_dimension_ = dim; }
    void setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper);
    
    // 设置验证函数
    void setStateValidityChecker(std::function<bool(const State&)> checker) {
        state_validity_checker_ = checker;
    }
    
    void setMotionValidator(std::function<bool(const State&, const State&)> validator) {
        motion_validator_ = validator;
    }
    
    // 主规划函数
    virtual bool solve(const State& start, const State& goal, 
                      std::vector<State>& path, double timeout);
    
    // 清除树
    void clear();
    
    // 获取节点数（用于统计）
    virtual size_t getNodeCount() const { 
        return motions_.size(); 
    }
    
protected:
    // 核心 RRT 函数
    State sampleRandomState();
    Motion* findNearestMotion(const State& state);
    double distance(const State& a, const State& b) const;
    State interpolate(const State& from, const State& to, double t) const;
    bool isGoalReached(const State& state, const State& goal, double threshold) const;
    void extractPath(Motion* goal_motion, std::vector<State>& path);
    void freeMemory();
    
    // KD-Tree 设置
    void setupNearestNeighbors();
    static double motionDistance(Motion* const& a, Motion* const& b);
    
    // 参数
    size_t state_dimension_;
    double max_distance_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;
    
    // 验证器
    std::function<bool(const State&)> state_validity_checker_;
    std::function<bool(const State&, const State&)> motion_validator_;
    
    // 树结构
    std::vector<Motion*> motions_;
    Motion* last_goal_motion_;
    
    // 随机数生成器
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_;
    
    // KD-Tree
    std::shared_ptr<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>> nn_;
};

} // namespace baseline_rrt

#endif // BASELINE_RRT_BASIC_RRT_H

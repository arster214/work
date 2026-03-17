#ifndef IMPROVED_RRT_PLANNER_IMPROVED_RRT_H
#define IMPROVED_RRT_PLANNER_IMPROVED_RRT_H

#include <vector>
#include <memory>
#include <random>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

// OMPL KD-Tree for fast nearest neighbor search
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

namespace improved_rrt_planner
{

// 简化的状态表示（完全模仿OMPL）
struct State
{
    std::vector<double> values;
    
    State() = default;
    explicit State(size_t dim) : values(dim, 0.0) {}
    State(const std::vector<double>& v) : values(v) {}
};

// Motion结构（完全模仿OMPL TRRT）
struct Motion
{
    State state;
    Motion* parent;        // 父节点指针（OMPL风格）
    double cost;           // 状态代价（用于T-RRT）
    
    Motion() : parent(nullptr), cost(0.0) {}
    explicit Motion(const State& s) : state(s), parent(nullptr), cost(0.0) {}
};

// RRT算法核心类（模仿OMPL TRRT，T-RRT部分可选）
class ImprovedRRT
{
public:
    ImprovedRRT();
    ~ImprovedRRT();
    
    // 基础RRT参数
    void setRange(double range) { max_distance_ = range; }
    void setGoalBias(double bias) { goal_bias_ = bias; }
    void setStateDimension(size_t dim) { state_dimension_ = dim; }
    void setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper);
    
    // T-RRT参数（可选，通过enable_trrt开关）
    void setEnableTRRT(bool enable) { enable_trrt_ = enable; }
    void setInitTemperature(double temp) { init_temperature_ = temp; temp_ = temp; }
    void setTempChangeFactor(double factor) { temp_change_factor_ = exp(factor); }
    void setCostThreshold(double threshold) { cost_threshold_ = threshold; }
    void setFrontierThreshold(double threshold) { frontier_threshold_ = threshold; }
    void setFrontierNodeRatio(double ratio) { frontier_node_ratio_ = ratio; }
    
    // RRT*参数（可选，通过enable_rrt_star开关）
    void setEnableRRTStar(bool enable) { enable_rrt_star_ = enable; }
    void setRewireRadius(double radius) { rewire_radius_ = radius; }
    // 双向Connect开关
    void setEnableConnect(bool enable) { enable_connect_ = enable; }
    
    // 设置状态验证函数
    void setStateValidityChecker(std::function<bool(const State&)> checker) {
        state_validity_checker_ = checker;
    }
    
    // 设置运动验证函数
    void setMotionValidator(std::function<bool(const State&, const State&)> validator) {
        motion_validator_ = validator;
    }
    
    // 设置状态代价函数（用于T-RRT）
    void setStateCostFunction(std::function<double(const State&)> cost_fn) {
        state_cost_function_ = cost_fn;
    }
    
    // 设置运动代价函数（用于T-RRT）
    void setMotionCostFunction(std::function<double(const State&, const State&)> cost_fn) {
        motion_cost_function_ = cost_fn;
    }
    
    // 设置 PlanningScene（用于代价计算）
    void setPlanningScene(const planning_scene::PlanningSceneConstPtr& scene,
                         const moveit::core::RobotModelConstPtr& model,
                         const std::string& group_name) {
        planning_scene_ = scene;
        robot_model_ = model;
        group_name_ = group_name;
    }
    
    // 规划主函数（完全模仿OMPL）
    bool solve(const State& start, const State& goal, 
               std::vector<State>& path, double timeout);
    
    // 清除树
    void clear();
    
    // 验证配置是否正确
    bool validateConfiguration() const;
    
private:
    // 核心RRT函数（完全模仿OMPL）
    State sampleRandomState();
    Motion* findNearestMotion(const State& state);
    double distance(const State& a, const State& b) const;
    State interpolate(const State& from, const State& to, double t) const;
    bool isGoalReached(const State& state, const State& goal, double threshold) const;
    void extractPath(Motion* goal_motion, std::vector<State>& path);
    void freeMemory();
    
    // T-RRT特有函数（可选）
    bool transitionTest(double motion_cost);
    bool minExpansionControl(double rand_motion_distance);
    
    // RRT*特有函数（可选）
    void rewireTree(Motion* new_motion, const std::vector<Motion*>& neighbors);
    std::vector<Motion*> getNearbyMotions(std::shared_ptr<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>> nn, Motion* motion, double radius);
    double pathCost(Motion* motion) const;
    
    // 参数
    size_t state_dimension_;
    double max_distance_;
    double goal_bias_;
    std::vector<double> joint_lower_limits_;
    std::vector<double> joint_upper_limits_;
    
    // T-RRT参数
    bool enable_trrt_;
    double temp_;
    double init_temperature_;
    double temp_change_factor_;
    double cost_threshold_;
    double best_cost_;
    double worst_cost_;
    double frontier_threshold_;
    double frontier_node_ratio_;
    double nonfrontier_count_;
    double frontier_count_;
    
    // RRT*参数
    bool enable_rrt_star_;
    double rewire_radius_;
    // 双向Connect参数
    bool enable_connect_;
    
    // 验证器和代价函数
    std::function<bool(const State&)> state_validity_checker_;
    std::function<bool(const State&, const State&)> motion_validator_;
    std::function<double(const State&)> state_cost_function_;
    std::function<double(const State&, const State&)> motion_cost_function_;
    
    // MoveIt 集成
    planning_scene::PlanningSceneConstPtr planning_scene_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::string group_name_;
    
    // 树结构（OMPL风格：指针列表）
    std::vector<Motion*> motions_;
    Motion* last_goal_motion_;
    
    // 随机数生成器
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_;
    
    // KD-Tree（OMPL风格）
    std::shared_ptr<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>> nn_;
    void setupNearestNeighbors();
    static double motionDistance(Motion* const& a, Motion* const& b);
};

} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_IMPROVED_RRT_H

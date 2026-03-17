// Auto-converted from improved_rrt_planner/improved_rrt.h
#ifndef DUAL_ARM_RRT_PLANNER_DUAL_ARM_RRT_H
#define DUAL_ARM_RRT_PLANNER_DUAL_ARM_RRT_H

#include <vector>
#include <memory>
#include <random>
#include <map>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

// OMPL KD-Tree for fast nearest neighbor search
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <geometry_msgs/Pose.h>
#include "dual_arm_rrt_planner/modules/module_base.h"

namespace dual_arm_planner
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
class DualArmRRT
{
public:
    struct HybridConnectConfig
    {
        bool enable_hybrid_connect{true};
        bool enable_dynamic_tree_balancing{true};
        bool enable_task_space_sobol_sampling{true};
        bool enable_goal_bias_sampling{true};
        bool enable_sparse_rejection{true};
        bool enable_aabb_apf{true};
        bool enable_variable_step{true};
        bool enable_trrt_acceptance{true};
        bool enable_greedy_connect{true};
        double sparse_rejection_ratio{0.15};
        double min_sparse_rejection_ratio{0.10};
        double max_sparse_rejection_ratio{0.20};
        double apf_joint_blend{0.35};
        double connect_distance_ratio{1.0};
        std::string ik_arm_group{""};
        std::string hybrid_param_namespace{"hybrid_connect"};
        Eigen::Vector3d task_space_min{-0.8, -0.8, 0.0};
        Eigen::Vector3d task_space_max{0.8, 0.8, 1.2};
        geometry_msgs::Pose ik_reference_pose;
    };

    DualArmRRT();
    ~DualArmRRT();
    
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
    void setHybridConnectConfig(const HybridConnectConfig& config) { hybrid_config_ = config; }
    void setObstacleAABBs(const std::vector<AABB>& aabbs) { obstacle_aabbs_ = aabbs; }
    
    // 规划主函数（完全模仿OMPL）
    bool solve(const State& start, const State& goal, 
               std::vector<State>& path, double timeout);
    
    // 清除树
    void clear();
    
    // 验证配置是否正确
    bool validateConfiguration() const;
    
    // --- ScheduleStream interfaces (public) ---
    // 给定物体 pose，生成候选抓取位姿（世界坐标系）
    std::vector<geometry_msgs::Pose> streamGenerateGrasps(const geometry_msgs::Pose& obj_pose);

    // 双臂正运动学
    std::pair<Eigen::Vector3d, Eigen::Vector3d> stateToTaskPoints(const State& state) const;
    
    // 双臂逆运动学与状态缝合
    bool mapTaskPointsToState(const Eigen::Vector3d& point_left, 
                              const Eigen::Vector3d& point_right, 
                              State& result_state, 
                              const State& seed_state) const;

    // 获取特定位置的障碍物信息 (保留旧接口做通用查询)
    ObstacleInfo buildObstacleInfo(const Eigen::Vector3d& task_point) const;
    
    // 给定抓取位姿和手臂组名（例如 "left_arm"/"right_arm"），尝试计算 IK
    // 返回 true 并填充 joints 当解算成功
    bool streamComputeIK(const geometry_msgs::Pose& pose, const std::string& arm_group, std::vector<double>& joints);
    
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
    bool createsCycle(Motion* child, Motion* new_parent) const;
    
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

    // modules (improved_rrt style modular enhancements)
    std::vector<modules::ModuleBasePtr> modules_;
    void initializeDefaultModules();

    void loadHybridConnectConfig();
    std::string resolveIKArmGroup() const;
    Eigen::Vector3d stateToTaskPoint(const State& state) const;
    geometry_msgs::Pose buildPoseFromTaskPoint(const Eigen::Vector3d& point) const;
    bool sampleTaskSpaceCandidate(State& sampled_state, geometry_msgs::Pose& sampled_pose, Eigen::Vector3d& sampled_task_point, bool& used_goal_bias);
    bool mapTaskPointToState(const Eigen::Vector3d& point, State& result_state, geometry_msgs::Pose& pose_out) const;
    double computeSparseThreshold() const;
    double computeStateTransitionCost(const State& from, const State& to) const;
    bool evaluateTRRTAcceptance(double parent_cost, double child_cost, double motion_cost);
    bool greedyConnectTrees(Motion* new_motion,
                            std::vector<Motion*>& target_tree,
                            std::shared_ptr<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>> target_nn,
                            Motion*& connect_target) const;
    uint32_t sobolIndexToGrayCode(uint32_t value) const;
    double sobolSample1D(uint32_t index, uint32_t dimension) const;
    Eigen::Vector3d nextSobolPoint();

    // cache the current planning goal for modules (used by GoalBiasModule etc.)
    State current_goal_state_;

    HybridConnectConfig hybrid_config_;
    std::vector<AABB> obstacle_aabbs_;
    uint32_t sobol_index_{1};
    std::array<uint32_t, 3> sobol_scramble_{{0u, 0u, 0u}};

    

    // --- 优化与诊断选项 ---
    int collision_check_limit_{3};
    bool enable_diagnostics_{false};

    // 诊断计数器（仅内部使用）
    size_t diag_parent_collision_checks_{0};
    size_t diag_parent_rejects_{0};
    size_t diag_rewire_collision_checks_{0};
    size_t diag_rewire_rejects_{0};

    // setters
    void setCollisionCheckLimit(int k) { collision_check_limit_ = std::max(1, k); }
    void setEnableDiagnostics(bool en) { enable_diagnostics_ = en; }
};

} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_DUAL_ARM_RRT_H

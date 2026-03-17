#ifndef DUAL_ARM_RRT_PLANNER_PLANNING_CONTEXT_H
#define DUAL_ARM_RRT_PLANNER_PLANNING_CONTEXT_H

#include <vector>
#include <limits>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

namespace dual_arm_planner
{

// 前向声明（完整定义在 dual_arm_rrt.h 中）
struct State;
struct Motion;

struct AABB
{
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
};

// 障碍物信息结构
struct ObstacleInfo
{
    std::vector<Eigen::Vector3d> nearby_obstacle_points;  // 附近障碍物点
    std::vector<AABB> aabbs;                              // AABB 障碍物
    double min_obstacle_distance;                          // 到最近障碍物的距离
    double obstacle_density;                               // 障碍物密度（障碍物体积/局部体积）
    Eigen::Vector3d closest_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d closest_gradient = Eigen::Vector3d::Zero();
    
    ObstacleInfo()
        : min_obstacle_distance(std::numeric_limits<double>::infinity())
        , obstacle_density(0.0)
    {}
};

// 规划上下文 - 在模块间共享的数据结构
struct PlanningContext
{
    // 状态信息
    const State* start_state;           // 起始状态（只读）
    const State* goal_state;            // 目标状态（只读）
    State* current_sample;              // 当前采样点（指针）
    State* new_node_candidate;          // 新节点候选（指针）
    const State* nearest_state;         // 最近节点状态
    const State* target_tree_nearest_state; // 目标树中最近节点状态
    int nearest_node_id;                // 最近节点索引
    
    // 步长信息（可被 Variable_Step_Module 修改）
    double step_size;                   // 当前步长
    double default_step_size;           // 默认步长
    
    // 有效性标志（模块可设置为 false 来拒绝节点）
    bool is_valid;
    
    // 代价信息
    double motion_cost;                 // 运动代价
    double accumulated_cost;            // 累积代价
    double parent_cost;                 // 父节点代价
    
    // 障碍物信息（由 RRT_Core 填充）
    ObstacleInfo obstacle_info;
    
    // 树引用（模块只读）
    // Note: motions_ in DualArmRRT is a std::vector<Motion*>, so the tree pointer
    // should reference a vector of Motion* to avoid conversions.
    const std::vector<Motion*>* tree;
    
    // 迭代信息
    int iteration;
    
    // 双臂独立的任务空间变量
    Eigen::Vector3d current_task_point_left;
    Eigen::Vector3d current_task_point_right;
    
    Eigen::Vector3d target_task_point_left;
    Eigen::Vector3d target_task_point_right;
    
    Eigen::Vector3d modified_task_point_left;
    Eigen::Vector3d modified_task_point_right;
    
    ObstacleInfo obstacle_info_left;
    ObstacleInfo obstacle_info_right;

    // 保留旧变量以兼容部分未修改代码，但在 APF 中主要使用上面的新变量
    Eigen::Vector3d sampled_task_point;
    Eigen::Vector3d apf_task_force;
    geometry_msgs::Pose sampled_pose;
    
    // 构造函数
    PlanningContext()
        : start_state(nullptr)
        , goal_state(nullptr)
        , current_sample(nullptr)
        , new_node_candidate(nullptr)
    , nearest_state(nullptr)
    , target_tree_nearest_state(nullptr)
        , nearest_node_id(-1)
        , step_size(0.5)
        , default_step_size(0.5)
        , is_valid(true)
        , motion_cost(0.0)
        , accumulated_cost(0.0)
        , parent_cost(0.0)
    , tree(nullptr)
        , iteration(0)
    {}
    
    // 重置（每次迭代开始时调用）
    void reset()
    {
        is_valid = true;
        motion_cost = 0.0;
        step_size = default_step_size;
        obstacle_info = ObstacleInfo();
        obstacle_info_left = ObstacleInfo();
        obstacle_info_right = ObstacleInfo();
        apf_task_force.setZero();
        modified_task_point_left.setZero();
        modified_task_point_right.setZero();
    }
};

} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_PLANNING_CONTEXT_H

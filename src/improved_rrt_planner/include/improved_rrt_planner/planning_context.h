#ifndef IMPROVED_RRT_PLANNER_PLANNING_CONTEXT_H
#define IMPROVED_RRT_PLANNER_PLANNING_CONTEXT_H

#include <vector>
#include <limits>
#include <Eigen/Dense>

namespace improved_rrt_planner
{

// 前向声明（完整定义在 improved_rrt.h 中）
struct State;
struct Motion;

// 障碍物信息结构
struct ObstacleInfo
{
    std::vector<Eigen::Vector3d> nearby_obstacle_points;  // 附近障碍物点
    double min_obstacle_distance;                          // 到最近障碍物的距离
    double obstacle_density;                               // 障碍物密度（障碍物体积/局部体积）
    
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
    int nearest_node_id;                // 最近节点索引
    
    // 步长信息（可被 Variable_Step_Module 修改）
    double step_size;                   // 当前步长
    double default_step_size;           // 默认步长
    
    // 有效性标志（模块可设置为 false 来拒绝节点）
    bool is_valid;
    
    // 代价信息
    double motion_cost;                 // 运动代价
    double accumulated_cost;            // 累积代价
    
    // 障碍物信息（由 RRT_Core 填充）
    ObstacleInfo obstacle_info;
    
    // 树引用（模块只读）
    const std::vector<Motion>* tree;
    
    // 迭代信息
    int iteration;
    
    // 构造函数
    PlanningContext()
        : start_state(nullptr)
        , goal_state(nullptr)
        , current_sample(nullptr)
        , new_node_candidate(nullptr)
        , nearest_node_id(-1)
        , step_size(0.5)
        , default_step_size(0.5)
        , is_valid(true)
        , motion_cost(0.0)
        , accumulated_cost(0.0)
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
    }
};

} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_PLANNING_CONTEXT_H

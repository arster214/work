#include "baseline_rrt/basic_rrt_star.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace baseline_rrt
{

BasicRRTStar::BasicRRTStar()
    : BasicRRT()
    , rewire_radius_(0.75)
{
    ROS_DEBUG("[BasicRRTStar] Constructor completed");
}

BasicRRTStar::~BasicRRTStar()
{
    ROS_DEBUG("[BasicRRTStar] Destructor called");
}

double BasicRRTStar::pathCost(Motion* motion) const
{
    // 在 BasicRRTStar 中，所有 Motion 都是 MotionStar
    MotionStar* motion_star = static_cast<MotionStar*>(motion);
    return motion_star->cost;
}

std::vector<Motion*> BasicRRTStar::getNearbyMotions(Motion* motion, double radius)
{
    std::vector<Motion*> neighbors;
    
    // 使用 k-nearest 作为近似，大幅增加搜索数量到 100
    std::size_t k = std::min(static_cast<std::size_t>(100), motions_.size());
    std::vector<Motion*> k_nearest;
    nn_->nearestK(motion, k, k_nearest);
    
    // 过滤出半径内的节点
    for (Motion* neighbor : k_nearest)
    {
        if (neighbor != motion && distance(motion->state, neighbor->state) <= radius)
        {
            neighbors.push_back(neighbor);
        }
    }
    
    return neighbors;
}

void BasicRRTStar::rewireTree(Motion* new_motion, const std::vector<Motion*>& neighbors)
{
    double new_motion_cost = pathCost(new_motion);
    
    for (Motion* neighbor : neighbors)
    {
        if (neighbor == new_motion || neighbor == new_motion->parent)
            continue;
        
        // 计算通过 new_motion 到达 neighbor 的代价
        double cost_through_new = new_motion_cost + distance(new_motion->state, neighbor->state);
        double current_cost = pathCost(neighbor);
        
        // 如果通过 new_motion 更优（任何改进都接受）
        if (cost_through_new < current_cost)
        {
            // 验证运动有效性
            if (motion_validator_ && motion_validator_(new_motion->state, neighbor->state))
            {
                // Rewire
                neighbor->parent = new_motion;
                
                // 更新 cost（关键！）
                MotionStar* neighbor_star = static_cast<MotionStar*>(neighbor);
                neighbor_star->cost = cost_through_new;
            }
        }
    }
}

bool BasicRRTStar::solve(const State& start, const State& goal, 
                        std::vector<State>& path, double timeout)
{
    ROS_INFO("[BasicRRTStar] solve() called - state_dimension=%zu", state_dimension_);
    
    // 验证参数
    if (state_dimension_ == 0)
    {
        ROS_ERROR("[BasicRRTStar] state_dimension is 0");
        return false;
    }
    
    if (joint_lower_limits_.size() != state_dimension_ || 
        joint_upper_limits_.size() != state_dimension_)
    {
        ROS_ERROR("[BasicRRTStar] Joint limits size mismatch");
        return false;
    }
    
    if (!state_validity_checker_ || !motion_validator_)
    {
        ROS_ERROR("[BasicRRTStar] Validity checkers not set");
        return false;
    }
    
    clear();
    
    // 添加起始状态（使用 MotionStar）
    auto* start_motion = new (std::nothrow) MotionStar(start);
    if (!start_motion)
    {
        ROS_ERROR("[BasicRRTStar] Failed to allocate memory for start motion");
        return false;
    }
    start_motion->cost = 0.0;  // 起点 cost 为 0
    
    motions_.push_back(start_motion);
    nn_->add(start_motion);
    
    ROS_INFO("[BasicRRTStar] Starting Basic RRT* planning");
    
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = start_time + ros::Duration(timeout);
    
    Motion* solution = nullptr;
    Motion* approx_solution = nullptr;
    double approx_difference = std::numeric_limits<double>::infinity();
    double goal_threshold = 0.02;
    
    int iteration = 0;
    double goal_bias = 0.05;  // Small goal bias to help convergence
    
    // 主循环 - 随机采样，带少量目标偏置
    while (ros::Time::now() < end_time)
    {
        ++iteration;
        
        // 1. 随机采样（5% 概率采样目标）
        State rand_state;
        if (uniform_dist_(rng_) < goal_bias)
        {
            rand_state = goal;
        }
        else
        {
            rand_state = sampleRandomState();
        }
        
        // 2. 找最近节点
        Motion* near_motion = findNearestMotion(rand_state);
        if (!near_motion)
        {
            ROS_ERROR("[BasicRRTStar] findNearestMotion returned nullptr");
            break;
        }
        
        // 3. 扩展
        double dist = distance(near_motion->state, rand_state);
        State new_state = rand_state;
        
        if (dist > max_distance_)
        {
            double t = max_distance_ / dist;
            new_state = interpolate(near_motion->state, rand_state, t);
        }
        
        // 4. 验证运动
        if (!motion_validator_ || !motion_validator_(near_motion->state, new_state))
            continue;
        
        // 5. 创建新节点（使用 MotionStar）
        auto* motion = new (std::nothrow) MotionStar(new_state);
        if (!motion)
        {
            ROS_ERROR("[BasicRRTStar] Failed to allocate memory for motion");
            continue;
        }
        
        motion->parent = near_motion;
        motion->cost = pathCost(near_motion) + distance(near_motion->state, new_state);
        
        // ========== RRT* Step 1: 选择最优父节点 ==========
        double search_radius = rewire_radius_ > 0.0 ? rewire_radius_ : max_distance_ * 5.0;
        std::vector<Motion*> neighbors = getNearbyMotions(motion, search_radius);
        
        // 限制邻居数量（增加到 50）
        if (neighbors.size() > 50)
        {
            neighbors.resize(50);
        }
        
        // 选择代价最小的父节点
        Motion* best_parent = near_motion;
        double best_cost = pathCost(near_motion) + distance(near_motion->state, motion->state);
        
        for (Motion* neighbor : neighbors)
        {
            if (neighbor == near_motion) continue;
            
            double cost_through_neighbor = pathCost(neighbor) + distance(neighbor->state, motion->state);
            if (cost_through_neighbor < best_cost)
            {
                // 验证运动有效性
                if (motion_validator_ && motion_validator_(neighbor->state, motion->state))
                {
                    best_parent = neighbor;
                    best_cost = cost_through_neighbor;
                }
            }
        }
        
        motion->parent = best_parent;
        motion->cost = best_cost;  // 更新 cost
        
        // 添加到树
        motions_.push_back(motion);
        nn_->add(motion);
        
        // ========== RRT* Step 2: Rewiring ==========
        // 每次都执行 rewiring 以优化路径
        rewireTree(motion, neighbors);
        
        // 6. 检查是否到达目标
        double dist_to_goal = distance(motion->state, goal);
        if (dist_to_goal < goal_threshold)
        {
            solution = motion;
            break;  // 找到解就停止
        }
        
        // 跟踪最接近目标的节点
        if (dist_to_goal < approx_difference)
        {
            approx_difference = dist_to_goal;
            approx_solution = motion;
        }
    }
    
    // 处理结果
    bool solved = false;
    bool approximate = false;
    
    if (solution == nullptr)
    {
        solution = approx_solution;
        approximate = true;
    }
    
    if (solution != nullptr)
    {
        last_goal_motion_ = solution;
        extractPath(solution, path);
        solved = true;
        
        double planning_time = (ros::Time::now() - start_time).toSec();
        ROS_INFO("[BasicRRTStar] Planning %s: %d iterations, %zu nodes, %.3f seconds%s",
                 solved ? "succeeded" : "failed",
                 iteration, motions_.size(), planning_time,
                 approximate ? " (approximate)" : "");
    }
    else
    {
        ROS_WARN("[BasicRRTStar] Planning failed: no solution found");
    }
    
    return solved;
}

} // namespace baseline_rrt

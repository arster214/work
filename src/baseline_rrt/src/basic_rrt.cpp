#include "baseline_rrt/basic_rrt.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace baseline_rrt
{

BasicRRT::BasicRRT()
    : state_dimension_(0)
    , max_distance_(0.5)
    , last_goal_motion_(nullptr)
    , rng_(std::random_device{}())
    , uniform_dist_(0.0, 1.0)
{
    setupNearestNeighbors();
    ROS_DEBUG("[BasicRRT] Constructor completed");
}

BasicRRT::~BasicRRT()
{
    ROS_DEBUG("[BasicRRT] Destructor called");
    freeMemory();
}

void BasicRRT::setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper)
{
    joint_lower_limits_ = lower;
    joint_upper_limits_ = upper;
}

void BasicRRT::clear()
{
    freeMemory();
    if (nn_)
        nn_->clear();
    last_goal_motion_ = nullptr;
    ROS_DEBUG("[BasicRRT] State cleared");
}

void BasicRRT::freeMemory()
{
    for (auto* motion : motions_)
    {
        delete motion;
    }
    motions_.clear();
}

void BasicRRT::setupNearestNeighbors()
{
    nn_ = std::make_shared<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>>();
    nn_->setDistanceFunction([this](Motion* const& a, Motion* const& b) {
        return motionDistance(a, b);
    });
    ROS_DEBUG("[BasicRRT] KD-Tree initialized");
}

double BasicRRT::motionDistance(Motion* const& a, Motion* const& b)
{
    double dist = 0.0;
    for (size_t i = 0; i < a->state.values.size(); ++i)
    {
        double diff = a->state.values[i] - b->state.values[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

double BasicRRT::distance(const State& a, const State& b) const
{
    double dist = 0.0;
    for (size_t i = 0; i < a.values.size(); ++i)
    {
        double diff = a.values[i] - b.values[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

State BasicRRT::sampleRandomState()
{
    State state(state_dimension_);
    for (size_t i = 0; i < state_dimension_; ++i)
    {
        state.values[i] = joint_lower_limits_[i] + 
                         uniform_dist_(rng_) * (joint_upper_limits_[i] - joint_lower_limits_[i]);
    }
    return state;
}

Motion* BasicRRT::findNearestMotion(const State& state)
{
    if (!nn_ || motions_.empty())
    {
        ROS_ERROR("[BasicRRT] findNearestMotion called with empty tree");
        return nullptr;
    }
    
    try
    {
        Motion temp_motion(state);
        return nn_->nearest(&temp_motion);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[BasicRRT] Exception in findNearestMotion: %s", e.what());
        return nullptr;
    }
}

State BasicRRT::interpolate(const State& from, const State& to, double t) const
{
    State result(state_dimension_);
    for (size_t i = 0; i < state_dimension_; ++i)
    {
        result.values[i] = from.values[i] + t * (to.values[i] - from.values[i]);
    }
    return result;
}

bool BasicRRT::isGoalReached(const State& state, const State& goal, double threshold) const
{
    return distance(state, goal) < threshold;
}

void BasicRRT::extractPath(Motion* goal_motion, std::vector<State>& path)
{
    path.clear();
    Motion* current = goal_motion;
    while (current != nullptr)
    {
        path.push_back(current->state);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
}

bool BasicRRT::solve(const State& start, const State& goal, 
                    std::vector<State>& path, double timeout)
{
    ROS_INFO("[BasicRRT] solve() called - state_dimension=%zu", state_dimension_);
    
    // 验证参数
    if (state_dimension_ == 0)
    {
        ROS_ERROR("[BasicRRT] state_dimension is 0");
        return false;
    }
    
    if (joint_lower_limits_.size() != state_dimension_ || 
        joint_upper_limits_.size() != state_dimension_)
    {
        ROS_ERROR("[BasicRRT] Joint limits size mismatch");
        return false;
    }
    
    if (!state_validity_checker_ || !motion_validator_)
    {
        ROS_ERROR("[BasicRRT] Validity checkers not set");
        return false;
    }
    
    clear();
    
    // 添加起始状态
    auto* start_motion = new (std::nothrow) Motion(start);
    if (!start_motion)
    {
        ROS_ERROR("[BasicRRT] Failed to allocate memory for start motion");
        return false;
    }
    
    motions_.push_back(start_motion);
    nn_->add(start_motion);
    
    ROS_INFO("[BasicRRT] Starting Basic RRT planning");
    
    ros::Time start_time = ros::Time::now();
    ros::Time end_time = start_time + ros::Duration(timeout);
    
    Motion* solution = nullptr;
    Motion* approx_solution = nullptr;
    double approx_difference = std::numeric_limits<double>::infinity();
    double goal_threshold = 0.02;
    
    int iteration = 0;
    double goal_bias = 0.03;  // Small goal bias to help convergence
    
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
            ROS_ERROR("[BasicRRT] findNearestMotion returned nullptr");
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
        
        // 5. 添加新节点
        auto* motion = new (std::nothrow) Motion(new_state);
        if (!motion)
        {
            ROS_ERROR("[BasicRRT] Failed to allocate memory for motion");
            continue;
        }
        
        motion->parent = near_motion;
        motions_.push_back(motion);
        nn_->add(motion);
        
        // 6. 检查是否到达目标
        double dist_to_goal = distance(motion->state, goal);
        if (dist_to_goal < goal_threshold)
        {
            solution = motion;
            break;
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
        ROS_INFO("[BasicRRT] Planning %s: %d iterations, %zu nodes, %.3f seconds%s",
                 solved ? "succeeded" : "failed",
                 iteration, motions_.size(), planning_time,
                 approximate ? " (approximate)" : "");
    }
    else
    {
        ROS_WARN("[BasicRRT] Planning failed: no solution found");
    }
    
    return solved;
}

} // namespace baseline_rrt

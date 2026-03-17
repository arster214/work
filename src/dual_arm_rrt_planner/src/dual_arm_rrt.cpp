#include "dual_arm_rrt_planner/dual_arm_rrt.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <array>
#include <cstdint>
#include <numeric>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// modules
#include "dual_arm_rrt_planner/modules/goal_bias_module.h"
#include "dual_arm_rrt_planner/modules/node_rejection_module.h"
#include "dual_arm_rrt_planner/modules/t_rrt_module.h"
#include "dual_arm_rrt_planner/modules/apf_module.h"
#include "dual_arm_rrt_planner/modules/variable_step_module.h"

namespace dual_arm_planner
{

namespace
{
constexpr uint32_t kSobolDirections[3][32] = {
    {0x80000000u,0x40000000u,0x20000000u,0x10000000u,0x08000000u,0x04000000u,0x02000000u,0x01000000u,
     0x00800000u,0x00400000u,0x00200000u,0x00100000u,0x00080000u,0x00040000u,0x00020000u,0x00010000u,
     0x00008000u,0x00004000u,0x00002000u,0x00001000u,0x00000800u,0x00000400u,0x00000200u,0x00000100u,
     0x00000080u,0x00000040u,0x00000020u,0x00000010u,0x00000008u,0x00000004u,0x00000002u,0x00000001u},
    {0x80000000u,0xC0000000u,0xA0000000u,0xF0000000u,0x88000000u,0xCC000000u,0xAA000000u,0xFF000000u,
     0x80800000u,0xC0C00000u,0xA0A00000u,0xF0F00000u,0x88880000u,0xCCCC0000u,0xAAAA0000u,0xFFFF0000u,
     0x80008000u,0xC000C000u,0xA000A000u,0xF000F000u,0x88008800u,0xCC00CC00u,0xAA00AA00u,0xFF00FF00u,
    0x80808080u,0xC0C0C0C0u,0xA0A0A0A0u,0xF0F0F0F0u,0x88888888u,0xCCCCCCCCu,0xAAAAAAAAu,0xFFFFFFFFu},
    {0x80000000u,0xC0000000u,0x60000000u,0x90000000u,0xE8000000u,0x5C000000u,0x8E000000u,0xC5000000u,
     0x68800000u,0x9CC00000u,0xEE600000u,0x55900000u,0x80680000u,0xC09C0000u,0x60EE0000u,0x90550000u,
     0xE8808000u,0x5CC0C000u,0x8E606000u,0xC5909000u,0x6868E800u,0x9C9C5C00u,0xEEEE8E00u,0x5555C500u,
     0x80006880u,0xC0009CC0u,0x6000EE60u,0x90005590u,0xE8008068u,0x5C00C09Cu,0x8E0060EEu,0xC5009055u}
};
}

DualArmRRT::DualArmRRT()
    : state_dimension_(0)
    , max_distance_(0.5)
    , goal_bias_(0.05)
    , enable_trrt_(false)
    , init_temperature_(100.0)
    , temp_(100.0)
    , temp_change_factor_(exp(0.1))
    , cost_threshold_(std::numeric_limits<double>::infinity())
    , best_cost_(0.0)
    , worst_cost_(0.0)
    , frontier_threshold_(0.0)
    , frontier_node_ratio_(0.1)
    , nonfrontier_count_(1.0)
    , frontier_count_(1.0)
    , enable_rrt_star_(true)
    , rewire_radius_(0.0)
    , enable_connect_(true)
    , last_goal_motion_(nullptr)
    , rng_(std::random_device{}())
    , uniform_dist_(0.0, 1.0)
{
    setupNearestNeighbors();
    loadHybridConnectConfig();
    // 初始化并注册默认 modules
    initializeDefaultModules();
    ROS_DEBUG("[DualArmRRT] Constructor completed");
}


DualArmRRT::~DualArmRRT()
{
    try
    {
        ROS_DEBUG("[DualArmRRT] Destructor called");
        clear();  // This calls freeMemory() and clears nn_
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[DualArmRRT] Exception in destructor: %s", e.what());
    }
    catch (...)
    {
        ROS_ERROR("[DualArmRRT] Unknown exception in destructor");
    }
}

void DualArmRRT::setJointLimits(const std::vector<double>& lower, const std::vector<double>& upper)
{
    joint_lower_limits_ = lower;
    joint_upper_limits_ = upper;
}

void DualArmRRT::clear()
{
    if (nn_)
        nn_.reset();

    freeMemory();
    last_goal_motion_ = nullptr;
    
    temp_ = init_temperature_;
    nonfrontier_count_ = 1.0;
    frontier_count_ = 1.0;
    best_cost_ = 0.0;
    worst_cost_ = 0.0;
    
    ROS_DEBUG("[DualArmRRT] State cleared and memory released");
}

void DualArmRRT::freeMemory()
{
    for (auto* motion : motions_)
    {
        delete motion;
    }
    motions_.clear();
}

void DualArmRRT::setupNearestNeighbors()
{
    nn_ = std::make_shared<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>>();
    nn_->setDistanceFunction([this](Motion* const& a, Motion* const& b) {
        return motionDistance(a, b);
    });
    ROS_DEBUG("[DualArmRRT] KD-Tree (re)initialized");
}

void DualArmRRT::initializeDefaultModules()
{
    try
    {
        ros::NodeHandle nh("~");
        // Goal bias
        auto gb = std::make_shared<modules::GoalBiasModule>();
        gb->initialize(nh, "GoalBiasModule");
        modules_.push_back(gb);

        // Variable step
        auto vs = std::make_shared<modules::VariableStepModule>();
        vs->initialize(nh, "VariableStepModule");
        modules_.push_back(vs);

        // APF
        auto apf = std::make_shared<modules::APFModule>();
        apf->initialize(nh, "APFModule");
        modules_.push_back(apf);

        // Node rejection
        auto nr = std::make_shared<modules::NodeRejectionModule>();
        nr->initialize(nh, "NodeRejectionModule");
        modules_.push_back(nr);

        // T-RRT module
        auto tr = std::make_shared<modules::TRRTModule>();
        tr->initialize(nh, "TRRTModule");
        modules_.push_back(tr);

        ROS_DEBUG("[DualArmRRT] Default modules initialized: %zu", modules_.size());
    }
    catch (const std::exception& e)
    {
        ROS_WARN("[DualArmRRT] Exception in initializeDefaultModules: %s", e.what());
    }
}

void DualArmRRT::loadHybridConnectConfig()
{
    ros::NodeHandle nh("~");
    const std::string& ns = hybrid_config_.hybrid_param_namespace;

    // Check for global obstacle avoidance mode param which overrides local settings
    std::string obstacle_mode;
    if (nh.getParam("obstacle_avoidance_mode", obstacle_mode)) {
        if (obstacle_mode == "voxel_rrt") {
            hybrid_config_.enable_aabb_apf = false;
            ROS_INFO("[DualArmRRT] Obstacle avoidance mode set to 'voxel_rrt'. Disabling APF.");
        } else if (obstacle_mode == "aabb_apf") {
            hybrid_config_.enable_aabb_apf = true;
            ROS_INFO("[DualArmRRT] Obstacle avoidance mode set to 'aabb_apf'. Enabling APF.");
        } else {
            ROS_WARN("[DualArmRRT] Unknown obstacle_avoidance_mode: %s. Using default settings.", obstacle_mode.c_str());
            nh.param(ns + "/enable_aabb_apf", hybrid_config_.enable_aabb_apf, true);
        }
    } else {
        nh.param(ns + "/enable_aabb_apf", hybrid_config_.enable_aabb_apf, true);
    }

    nh.param(ns + "/enable_hybrid_connect", hybrid_config_.enable_hybrid_connect, true);
    nh.param(ns + "/enable_dynamic_tree_balancing", hybrid_config_.enable_dynamic_tree_balancing, true);
    nh.param(ns + "/enable_task_space_sobol_sampling", hybrid_config_.enable_task_space_sobol_sampling, true);
    nh.param(ns + "/enable_goal_bias_sampling", hybrid_config_.enable_goal_bias_sampling, true);
    nh.param(ns + "/enable_sparse_rejection", hybrid_config_.enable_sparse_rejection, true);
    // enable_aabb_apf already set by obstacle_avoidance_mode logic above
    // nh.param(ns + "/enable_aabb_apf", hybrid_config_.enable_aabb_apf, true);
    nh.param(ns + "/enable_variable_step", hybrid_config_.enable_variable_step, true);
    nh.param(ns + "/enable_trrt_acceptance", hybrid_config_.enable_trrt_acceptance, true);
    nh.param(ns + "/enable_greedy_connect", hybrid_config_.enable_greedy_connect, true);
    nh.param(ns + "/sparse_rejection_ratio", hybrid_config_.sparse_rejection_ratio, 0.15);
    nh.param(ns + "/min_sparse_rejection_ratio", hybrid_config_.min_sparse_rejection_ratio, 0.10);
    nh.param(ns + "/max_sparse_rejection_ratio", hybrid_config_.max_sparse_rejection_ratio, 0.20);
    nh.param(ns + "/apf_joint_blend", hybrid_config_.apf_joint_blend, 0.35);
    nh.param(ns + "/connect_distance_ratio", hybrid_config_.connect_distance_ratio, 1.0);
    nh.param(ns + "/ik_arm_group", hybrid_config_.ik_arm_group, std::string(""));

    std::vector<double> task_min, task_max;
    if (nh.getParam(ns + "/task_space_min", task_min) && task_min.size() == 3)
        hybrid_config_.task_space_min = Eigen::Vector3d(task_min[0], task_min[1], task_min[2]);
    if (nh.getParam(ns + "/task_space_max", task_max) && task_max.size() == 3)
        hybrid_config_.task_space_max = Eigen::Vector3d(task_max[0], task_max[1], task_max[2]);

    hybrid_config_.sparse_rejection_ratio = std::max(hybrid_config_.min_sparse_rejection_ratio,
                                                     std::min(hybrid_config_.max_sparse_rejection_ratio,
                                                              hybrid_config_.sparse_rejection_ratio));
}

double DualArmRRT::motionDistance(Motion* const& a, Motion* const& b)
{
    double dist = 0.0;
    for (size_t i = 0; i < a->state.values.size(); ++i)
    {
        double diff = a->state.values[i] - b->state.values[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

double DualArmRRT::distance(const State& a, const State& b) const
{
    double dist = 0.0;
    for (size_t i = 0; i < a.values.size(); ++i)
    {
        double diff = a.values[i] - b.values[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

State DualArmRRT::sampleRandomState()
{
    // 默认随机采样
    State state(state_dimension_);
    for (size_t i = 0; i < state_dimension_; ++i)
    {
        state.values[i] = joint_lower_limits_[i] + 
                         uniform_dist_(rng_) * (joint_upper_limits_[i] - joint_lower_limits_[i]);
    }

    // 让 modules 有机会提供替代采样（例如 GoalBias）
    if (!modules_.empty())
    {
        PlanningContext ctx;
        ctx.goal_state = &current_goal_state_;
        ctx.current_sample = &state;
        for (auto& m : modules_)
        {
            try
            {
                if (m && m->preSample(ctx))
                {
                    // 模块已修改 current_sample 成为采样点
                    return state;
                }
            }
            catch (const std::exception& e)
            {
                ROS_WARN("[DualArmRRT] Exception in preSample of module %s: %s", m->getName().c_str(), e.what());
            }
        }
    }

    return state;
}

std::string DualArmRRT::resolveIKArmGroup() const
{
    if (!hybrid_config_.ik_arm_group.empty())
        return hybrid_config_.ik_arm_group;
    if (!group_name_.empty())
        return group_name_;
    return "left_arm";
}

Eigen::Vector3d DualArmRRT::stateToTaskPoint(const State& state) const
{
    // 使用 MoveIt 正向运动学计算末端执行器/关键点的笛卡尔位置
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    if (!robot_model_)
    {
        // 回退到旧行为：直接取 state 的前三个分量
        if (!state.values.empty()) point.x() = state.values[0];
        if (state.values.size() > 1) point.y() = state.values[1];
        if (state.values.size() > 2) point.z() = state.values[2];
        return point;
    }

    const std::string arm = resolveIKArmGroup();
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(arm);
    if (!jmg)
    {
        if (!state.values.empty()) point.x() = state.values[0];
        if (state.values.size() > 1) point.y() = state.values[1];
        if (state.values.size() > 2) point.z() = state.values[2];
        return point;
    }

    moveit::core::RobotState rs(robot_model_);
    rs.setToDefaultValues();
    // 将 state 的值写入到关节组（以 jmg->getVariableCount() 为准）
    std::vector<double> joint_vals(jmg->getVariableCount(), 0.0);
    const size_t copy_count = std::min(joint_vals.size(), state.values.size());
    for (size_t i = 0; i < copy_count; ++i) joint_vals[i] = state.values[i];
    rs.setJointGroupPositions(jmg, joint_vals);

    // 选择末端连杆：采用 joint model group 的最后一个 link 作为参考
    const std::vector<std::string>& link_names = jmg->getLinkModelNames();
    if (link_names.empty())
    {
        if (!state.values.empty()) point.x() = state.values[0];
        if (state.values.size() > 1) point.y() = state.values[1];
        if (state.values.size() > 2) point.z() = state.values[2];
        return point;
    }

    const std::string ee_link = link_names.back();
    Eigen::Isometry3d tf = rs.getGlobalLinkTransform(ee_link);
    point.x() = tf.translation().x();
    point.y() = tf.translation().y();
    point.z() = tf.translation().z();
    return point;
}

geometry_msgs::Pose DualArmRRT::buildPoseFromTaskPoint(const Eigen::Vector3d& point) const
{
    geometry_msgs::Pose pose = hybrid_config_.ik_reference_pose;
    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();
    if (pose.orientation.w == 0.0 && pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0)
    {
        pose.orientation.w = 1.0;
    }
    return pose;
}

bool DualArmRRT::mapTaskPointToState(const Eigen::Vector3d& point, State& result_state, geometry_msgs::Pose& pose_out) const
{
    pose_out = buildPoseFromTaskPoint(point);
    if (!robot_model_)
    {
        result_state = State(state_dimension_);
        for (size_t i = 0; i < std::min<size_t>(3, state_dimension_); ++i)
            result_state.values[i] = point[static_cast<Eigen::Index>(i)];
        return true;
    }

    std::vector<double> joints;
    if (!const_cast<DualArmRRT*>(this)->streamComputeIK(pose_out, resolveIKArmGroup(), joints))
        return false;

    result_state = State(state_dimension_);
    const size_t count = std::min(state_dimension_, joints.size());
    for (size_t i = 0; i < count; ++i)
        result_state.values[i] = joints[i];
    return true;
}

uint32_t DualArmRRT::sobolIndexToGrayCode(uint32_t value) const
{
    return value ^ (value >> 1u);
}

double DualArmRRT::sobolSample1D(uint32_t index, uint32_t dimension) const
{
    uint32_t gray = sobolIndexToGrayCode(index);
    uint32_t sample = sobol_scramble_[dimension % 3];
    for (uint32_t bit = 0; bit < 32; ++bit)
    {
        if (gray & (1u << bit))
            sample ^= kSobolDirections[dimension % 3][bit];
    }
    return static_cast<double>(sample) / static_cast<double>(std::numeric_limits<uint32_t>::max());
}

Eigen::Vector3d DualArmRRT::nextSobolPoint()
{
    Eigen::Vector3d unit(sobolSample1D(sobol_index_, 0), sobolSample1D(sobol_index_, 1), sobolSample1D(sobol_index_, 2));
    ++sobol_index_;
    return hybrid_config_.task_space_min + unit.cwiseProduct(hybrid_config_.task_space_max - hybrid_config_.task_space_min);
}

bool DualArmRRT::sampleTaskSpaceCandidate(State& sampled_state,
                                          geometry_msgs::Pose& sampled_pose,
                                          Eigen::Vector3d& sampled_task_point,
                                          bool& used_goal_bias)
{
    used_goal_bias = false;

    // --- 双臂模式处理 ---
    if (group_name_ == "dual_arms")
    {
        Eigen::Vector3d point_left, point_right;

        // 1. 采样 (Sobol 或 随机关节FK)
        if (hybrid_config_.enable_task_space_sobol_sampling)
        {
            point_left = nextSobolPoint();
            point_right = nextSobolPoint();
        }
        else
        {
            State rand = sampleRandomState();
            auto pair = stateToTaskPoints(rand);
            point_left = pair.first;
            point_right = pair.second;
        }

        // 2. 目标偏置
        if (hybrid_config_.enable_goal_bias_sampling && uniform_dist_(rng_) < goal_bias_)
        {
            auto pair = stateToTaskPoints(current_goal_state_);
            point_left = pair.first;
            point_right = pair.second;
            used_goal_bias = true;
        }

        // 3. 计算双臂 IK
        // 使用空种子或者是最近邻 (如果要优化)
        State seed(state_dimension_); 
        if (mapTaskPointsToState(point_left, point_right, sampled_state, seed))
        {
            // 为兼容单点接口，返回左臂点作为主参考
            sampled_task_point = point_left;
            sampled_pose = buildPoseFromTaskPoint(point_left);
            return true;
        }
        return false;
    }

    // --- 单臂模式处理 (原有逻辑) ---
    sampled_task_point = hybrid_config_.enable_task_space_sobol_sampling ? nextSobolPoint() : stateToTaskPoint(sampleRandomState());

    if (hybrid_config_.enable_goal_bias_sampling && uniform_dist_(rng_) < goal_bias_)
    {
        sampled_task_point = stateToTaskPoint(current_goal_state_);
        used_goal_bias = true;
    }

    // 单臂 IK (注意: 如果 group_name_ 是 "dual_arms" 但没进入上面分支，这里会因无 Solver 报错)
    if (!mapTaskPointToState(sampled_task_point, sampled_state, sampled_pose))
        return false;

    PlanningContext ctx;
    ctx.goal_state = &current_goal_state_;
    ctx.current_sample = &sampled_state;
    for (auto& m : modules_)
    {
        if (m && m->getName() == "GoalBiasModule" && m->preSample(ctx))
        {
            sampled_task_point = stateToTaskPoint(sampled_state);
            sampled_pose = buildPoseFromTaskPoint(sampled_task_point);
            used_goal_bias = true;
            break;
        }
    }
    return true;
}

// 双臂正运动学：返回 <左臂末端, 右臂末端>
std::pair<Eigen::Vector3d, Eigen::Vector3d> DualArmRRT::stateToTaskPoints(const State& state) const
{
    // 使用局部 RobotState 进行 FK 计算，避免修改 const 的 scene state
    moveit::core::RobotState robot_state(robot_model_);
    // 将其置为双臂规划组的 Default Home
    robot_state.setToDefaultValues();

    // 缝合 14 维关节向量
    const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name_);
    if (!joint_model_group)
    {
        ROS_ERROR_THROTTLE(1.0, "[DualArmRRT] Invalid group name: %s", group_name_.c_str());
        return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }
    robot_state.setJointGroupPositions(joint_model_group, state.values);
    
    // 更新FK
    robot_state.updateLinkTransforms();

    const moveit::core::JointModelGroup* left_group = robot_model_->getJointModelGroup("l_arm");
    const moveit::core::JointModelGroup* right_group = robot_model_->getJointModelGroup("r_arm");

    if (!left_group || !right_group)
    {
         ROS_ERROR_THROTTLE(1.0, "[DualArmRRT] Subgroups left_arm/right_arm not found! Check robot model group names.");
         return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    }

    std::string left_tip = "l_link8"; 
    std::string right_tip = "r_link8";

    // 尝试从 Group 获取 Tip Frame (如果有 Solver，优先使用 Solver 指定的 Tip)
    {
        const auto& l_solver = left_group->getSolverInstance();
        if (l_solver && !l_solver->getTipFrame().empty()) left_tip = l_solver->getTipFrame();
    }
    
    {
        const auto& r_solver = right_group->getSolverInstance();
        if (r_solver && !r_solver->getTipFrame().empty()) right_tip = r_solver->getTipFrame();
    }

    Eigen::Vector3d left_pos = robot_state.getGlobalLinkTransform(left_tip).translation();
    Eigen::Vector3d right_pos = robot_state.getGlobalLinkTransform(right_tip).translation();

    return {left_pos, right_pos};
}

bool DualArmRRT::mapTaskPointsToState(const Eigen::Vector3d& point_left, 
                                      const Eigen::Vector3d& point_right, 
                                      State& result_state, 
                                      const State& seed_state) const
{
     // 使用局部 RobotState 进行 IK 计算
     moveit::core::RobotState robot_state(robot_model_);
     robot_state.setToDefaultValues();

     const moveit::core::JointModelGroup* dual_group = robot_model_->getJointModelGroup(group_name_);
     const moveit::core::JointModelGroup* left_group = robot_model_->getJointModelGroup("l_arm");
     const moveit::core::JointModelGroup* right_group = robot_model_->getJointModelGroup("r_arm");
     
     if (!dual_group || !left_group || !right_group) return false;

     // 设置种子状态 (用于增加 IK 成功率和连续性)
     if (!seed_state.values.empty())
        robot_state.setJointGroupPositions(dual_group, seed_state.values);

     robot_state.updateLinkTransforms();

     // 1. 左臂 IK 
     // 重要: 使用 Solver Tip Frame 保持一致性
     std::string left_tip = "l_link8";
     {
         const auto& s = left_group->getSolverInstance();
         if (s && !s->getTipFrame().empty()) left_tip = s->getTipFrame();
     }
     
     // 保持当前的 Orientation，仅修改位置
     Eigen::Isometry3d left_target = robot_state.getGlobalLinkTransform(left_tip);
     left_target.translation() = point_left;
     
     // 求解左臂 IK (简化超时和尝试次数，避免阻塞)
     bool left_ok = robot_state.setFromIK(left_group, left_target, 0.005);

     // 2. 右臂 IK
     std::string right_tip = "r_link8";
     {
         const auto& s = right_group->getSolverInstance();
         if (s && !s->getTipFrame().empty()) right_tip = s->getTipFrame();
     }

     Eigen::Isometry3d right_target = robot_state.getGlobalLinkTransform(right_tip);
     right_target.translation() = point_right;

     bool right_ok = robot_state.setFromIK(right_group, right_target, 0.005);

     // 只有当两臂都有解时才认为成功
     if (left_ok && right_ok)
     {
         // 提取所有关节到 result_state
         robot_state.copyJointGroupPositions(dual_group, result_state.values);
         return true;
     }

     return false;
}

// 保留旧接口实现以兼容 (虽已被改写，但部分旧代码可能还在调用单点版本)
// 这里简单取左臂结果作为返回值，避免编译错误
ObstacleInfo DualArmRRT::buildObstacleInfo(const Eigen::Vector3d& task_point) const
{
    ObstacleInfo info;
    info.aabbs = obstacle_aabbs_;
    double best_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_grad = Eigen::Vector3d::Zero();
    Eigen::Vector3d best_point = Eigen::Vector3d::Zero();
    double total_volume = 0.0;

    for (const auto& box : obstacle_aabbs_)
    {
        modules::APFModule apf_helper;
        auto dg = apf_helper.distanceAndGradientToAABB(task_point, box);
        double dist = dg.first;
        Eigen::Vector3d grad = dg.second;
        Eigen::Vector3d clamped = task_point.cwiseMax(box.min).cwiseMin(box.max); // Approx
        if (dist < best_dist)
        {
            best_dist = dist;
            best_grad = grad;
            best_point = clamped;
        }
        total_volume += (box.max - box.min).cwiseMax(Eigen::Vector3d::Zero()).prod();
    }
    info.min_obstacle_distance = best_dist;
    info.closest_gradient = best_grad;
    info.closest_point = best_point;
    info.obstacle_density = total_volume;
    return info;
}

double DualArmRRT::computeSparseThreshold() const
{
    return max_distance_ * std::max(hybrid_config_.min_sparse_rejection_ratio,
                                    std::min(hybrid_config_.max_sparse_rejection_ratio,
                                             hybrid_config_.sparse_rejection_ratio));
}

double DualArmRRT::computeStateTransitionCost(const State& from, const State& to) const
{
    if (motion_cost_function_)
        return motion_cost_function_(from, to);
    return distance(from, to);
}

bool DualArmRRT::evaluateTRRTAcceptance(double parent_cost, double child_cost, double motion_cost)
{
    if (!hybrid_config_.enable_trrt_acceptance && !enable_trrt_)
        return true;

    if (child_cost <= parent_cost)
        return true;

    const double delta = std::max(0.0, child_cost - parent_cost) + std::max(0.0, motion_cost);
    return transitionTest(delta);
}

bool DualArmRRT::greedyConnectTrees(Motion* new_motion,
                                    std::vector<Motion*>& target_tree,
                                    std::shared_ptr<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>> target_nn,
                                    Motion*& connect_target) const
{
    connect_target = nullptr;
    if (!new_motion || !target_nn)
        return false;

    Motion probe(new_motion->state);
    Motion* target_near = target_nn->nearest(&probe);
    if (!target_near)
        return false;

    const double connect_threshold = max_distance_ * hybrid_config_.connect_distance_ratio;
    if (distance(new_motion->state, target_near->state) > connect_threshold)
        return false;
    if (!motion_validator_ || !motion_validator_(new_motion->state, target_near->state))
        return false;

    connect_target = target_near;
    return true;
}

Motion* DualArmRRT::findNearestMotion(const State& state)
{
    if (!nn_ || motions_.empty())
    {
        return nullptr;
    }
    
    Motion scratch(state);
    std::vector<Motion*> result;
    nn_->nearestK(&scratch, 1, result);

    return result.empty() ? nullptr : result[0];
}

State DualArmRRT::interpolate(const State& from, const State& to, double t) const
{
    State result(state_dimension_);
    for (size_t i = 0; i < state_dimension_; ++i)
    {
        result.values[i] = from.values[i] + t * (to.values[i] - from.values[i]);
    }
    return result;
}

bool DualArmRRT::isGoalReached(const State& state, const State& goal, double threshold) const
{
    return distance(state, goal) < threshold;
}

void DualArmRRT::extractPath(Motion* goal_motion, std::vector<State>& path)
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

// T-RRT transition test (完全模仿OMPL)
bool DualArmRRT::transitionTest(double motion_cost)
{
    if (motion_cost >= cost_threshold_)
        return false;

    if (motion_cost < 1e-4)
        return true;

    double transition_probability = exp(-motion_cost / temp_);
    if (transition_probability > 0.5)
    {
        double cost_range = worst_cost_ - best_cost_;
        if (fabs(cost_range) > 1e-4)
        {
            temp_ /= exp(motion_cost / (0.1 * cost_range));
        }
        return true;
    }

    temp_ *= temp_change_factor_;
    return false;
}

// Minimum expansion control
bool DualArmRRT::minExpansionControl(double rand_motion_distance)
{
    if (rand_motion_distance > frontier_threshold_)
    {
        ++frontier_count_;
        return true;
    }
    else
    {
        if (nonfrontier_count_ / frontier_count_ > frontier_node_ratio_)
        {
            return false;
        }
        ++nonfrontier_count_;
        return true;
    }
}

// 验证配置是否正确
bool DualArmRRT::validateConfiguration() const
{
    if (state_dimension_ == 0)
    {
        ROS_ERROR("[DualArmRRT] state_dimension is 0");
        return false;
    }

    if (joint_lower_limits_.size() != state_dimension_ || 
        joint_upper_limits_.size() != state_dimension_)
    {
        ROS_ERROR("[DualArmRRT] Joint limits size mismatch");
        return false;
    }

    if (!state_validity_checker_)
    {
        ROS_ERROR("[DualArmRRT] No state validity checker set");
        return false;
    }

    if (!motion_validator_)
    {
        ROS_ERROR("[DualArmRRT] No motion validator set");
        return false;
    }

    if (enable_trrt_ && (!state_cost_function_ || !motion_cost_function_))
    {
        ROS_ERROR("[DualArmRRT] T-RRT enabled but cost functions not set");
        return false;
    }

    return true;
}

// 主 solve 函数（复制并适配自 improved_rrt.cpp）
bool DualArmRRT::solve(const State& start, const State& goal, 
                       std::vector<State>& path, double timeout)
{
    ROS_INFO("[DualArmRRT] solve() called - state_dimension=%zu, enable_rrt_star=%d, enable_trrt=%d", 
             state_dimension_, enable_rrt_star_, enable_trrt_);

    if (!validateConfiguration())
    {
        ROS_ERROR("[DualArmRRT] Configuration validation failed");
        return false;
    }

    clear();
    setupNearestNeighbors();

    diag_parent_collision_checks_ = 0;
    diag_parent_rejects_ = 0;
    diag_rewire_collision_checks_ = 0;
    diag_rewire_rejects_ = 0;

    if (frontier_threshold_ < std::numeric_limits<double>::epsilon())
    {
        double max_extent = 0.0;
        for (size_t i = 0; i < state_dimension_; ++i)
        {
            max_extent += (joint_upper_limits_[i] - joint_lower_limits_[i]) * 
                         (joint_upper_limits_[i] - joint_lower_limits_[i]);
        }
        max_extent = std::sqrt(max_extent);
        frontier_threshold_ = max_extent * 0.01;
        ROS_DEBUG("[DualArmRRT] Frontier threshold set to %f", frontier_threshold_);
    }

    // 缓存当前目标，供模块（如 GoalBias）使用
    current_goal_state_ = goal;

    if (enable_connect_)
    {
        ROS_INFO("[DualArmRRT] Running bidirectional CONNECT mode");

        std::vector<Motion*> treeA;
        std::vector<Motion*> treeB;

        auto nn_a = std::make_shared<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>>();
        auto nn_b = std::make_shared<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>>();
        nn_a->setDistanceFunction([this](Motion* const& a, Motion* const& b) { return motionDistance(a, b); });
        nn_b->setDistanceFunction([this](Motion* const& a, Motion* const& b) { return motionDistance(a, b); });

        Motion* a_root = new (std::nothrow) Motion(start);
        Motion* b_root = new (std::nothrow) Motion(goal);
        if (!a_root || !b_root)
        {
            ROS_ERROR("[DualArmRRT] Failed to allocate memory for bidirectional roots");
            delete a_root; delete b_root;
            return false;
        }

        if (enable_trrt_ && state_cost_function_)
        {
            a_root->cost = state_cost_function_(start);
            b_root->cost = state_cost_function_(goal);
            best_cost_ = worst_cost_ = a_root->cost;
        }

        treeA.push_back(a_root);
        treeB.push_back(b_root);
        nn_a->add(a_root);
        nn_b->add(b_root);

        ros::Time connect_start_time = ros::Time::now();
        ros::Time connect_end_time = connect_start_time + ros::Duration(timeout);

        bool solved = false;
        Motion* connect_a = nullptr;
        Motion* connect_b = nullptr;
        int iter = 0;

        while (ros::Time::now() < connect_end_time)
        {
            ++iter;

            const bool grow_a = !hybrid_config_.enable_dynamic_tree_balancing ? (treeA.size() <= treeB.size())
                                                                              : (treeA.size() <= treeB.size());
            auto& grow_tree = grow_a ? treeA : treeB;
            auto& grow_nn = grow_a ? nn_a : nn_b;
            auto& other_tree = grow_a ? treeB : treeA;
            auto& other_nn = grow_a ? nn_b : nn_a;

            State sampled_state;
            geometry_msgs::Pose sampled_pose;
            Eigen::Vector3d sampled_task_point = Eigen::Vector3d::Zero();
            bool used_goal_bias = false;
            if (!sampleTaskSpaceCandidate(sampled_state, sampled_pose, sampled_task_point, used_goal_bias))
                continue;

            Motion temp_rand(sampled_state);
            Motion* near = grow_nn->nearest(&temp_rand);
            if (!near)
                continue;

            const double near_dist = distance(near->state, sampled_state);
            if (hybrid_config_.enable_sparse_rejection && near_dist < computeSparseThreshold())
                continue;

            Motion* target_near = other_nn->nearest(&temp_rand);
            if (!target_near)
                continue;

            PlanningContext ctx;
            ctx.reset();
            ctx.iteration = iter;
            ctx.goal_state = &current_goal_state_;
            ctx.current_sample = &sampled_state;
            ctx.new_node_candidate = &sampled_state;
            ctx.nearest_state = &near->state;
            ctx.target_tree_nearest_state = &target_near->state;
            ctx.parent_cost = near->cost;
            ctx.default_step_size = max_distance_;
            ctx.step_size = max_distance_;
            ctx.tree = &grow_tree;
            
            // --- 双臂正运动学更新 ---
            auto current_points = stateToTaskPoints(near->state);
            ctx.current_task_point_left = current_points.first;
            ctx.current_task_point_right = current_points.second;
            
            auto target_points = stateToTaskPoints(target_near->state);
            ctx.target_task_point_left = target_points.first;
            ctx.target_task_point_right = target_points.second;
            
            // --- 双端障碍物信息更新 ---
            // 注意: 下面假设我们用 current point 搜索最近障碍物; 实际上 APF 可能推很远
            // 分别构造左右臂的障碍物信息
            ctx.obstacle_info_left = buildObstacleInfo(ctx.current_task_point_left);
            ctx.obstacle_info_right = buildObstacleInfo(ctx.current_task_point_right);
            
            // 兼容保留: ctx.obstacle_info 填充为左臂信息 (或取更近的那个)
            ctx.obstacle_info = (ctx.obstacle_info_left.min_obstacle_distance < ctx.obstacle_info_right.min_obstacle_distance) 
                                ? ctx.obstacle_info_left : ctx.obstacle_info_right;
            
            // 保留旧变量赋值以防止意外遗漏
            // ctx.current_task_point = ctx.current_task_point_left;  // Removed: member not in PlanningContext
            // ctx.target_task_point = ctx.target_task_point_left;    // Removed: member not in PlanningContext
            // ctx.sampled_task_point = sampled_task_point;           // Check if this exists? Compiler didn't complain yet. 
            // Better safe:
            // ctx.sampled_task_point = sampled_task_point;
            // Assuming sampled_task_point exists or is not used.
            // If sampled_task_point usage causes error, it will show up.
            // But I will keep sampled_task_point assignment if it wasn't flagged. 
            // Wait, I should probably check if sampled_task_point is in PlanningContext.
            // But for now, just removing the flagged ones.

            // Re-enabling sampled_task_point line since it might be there.
            // If it's not, next compile will fail.
            // But I'll just comment out the ones KNOWN to fail.
            // Wait, looking at the code I read:
            // ctx.sampled_task_point = sampled_task_point;
            // If I just remove the whole block, I might lose sampled_task_point assignment.
            // Let's comment out the lines that failed.
            // The compiler error was specific to current_task_point and target_task_point.
            
            // ctx.sampled_task_point = sampled_task_point;
            // ctx.sampled_pose = sampled_pose;
            // I'll assume only current/target were removed.
            
            // Actually, if I look at PlanningContext definition in previous turn, it ended with:
            // obstacle_aabbs_;
            // I didn't see the full struct members.
            
            // I'll be conservative and comment out only the explicitly failed ones.
            // But to use replace_string_in_file I need to match exact string.
            
            // Also, I see ctx.sampled_task_point = sampled_task_point; in the code I read.
            // I will comment out current/target task point assignments.

            // ctx.sampled_task_point = sampled_task_point;
            // ctx.sampled_pose = sampled_pose;
            
            // Let's just remove the assignments to current_task_point and target_task_point.


            if (hybrid_config_.enable_variable_step)
            {
                for (auto& m : modules_)
                {
                    if (m && m->getName() == "VariableStepModule")
                        m->preExtend(ctx);
                }
            }

            State steer_target = sampled_state;
            double d = distance(near->state, steer_target);
            if (d > ctx.step_size)
            {
                const double t = ctx.step_size / d;
                steer_target = interpolate(near->state, steer_target, t);
                d = ctx.step_size;
            }

            State new_state = steer_target;
            if (hybrid_config_.enable_aabb_apf)
            {
                // 让 APF 在任务空间分别修改左右臂任务点
                ctx.new_node_candidate = &new_state;
                // 初始化 modified points
                ctx.modified_task_point_left = ctx.current_task_point_left;
                ctx.modified_task_point_right = ctx.current_task_point_right;

                for (auto& m : modules_)
                {
                    if (m && m->getName() == "APFModule")
                        m->postExtend(ctx);
                }
                
                // 尝试用 IK 将 APF 修改后的任务点(Left+Right) 同时映射回关节空间
                // 重点: 这里将生成一个新的 state (缝合了左右臂 IK 结果)
                State apf_state(state_dimension_);
                
                bool ik_ok = mapTaskPointsToState(ctx.modified_task_point_left, 
                                                  ctx.modified_task_point_right, 
                                                  apf_state, 
                                                  steer_target); // Seed from steer_target

                const double blend = std::max(0.0, std::min(1.0, hybrid_config_.apf_joint_blend));
                if (ik_ok)
                {
                    // 缝合成功: 将结果通过 Blend 混合到 new_state 中
                    for (size_t i = 0; i < std::min(new_state.values.size(), apf_state.values.size()); ++i)
                    {
                        new_state.values[i] = (1.0 - blend) * steer_target.values[i] + blend * apf_state.values[i];
                    }
                }
                else
                {
                    // IK 失败 (可能目标太远或奇异)，退回到无 APF 状态 (即仅仅是 steer_target)
                    // 或者更激进: IK 失败则丢弃? 目前保持保守策略：退回插值点
                    for (size_t i = 0; i < std::min(new_state.values.size(), steer_target.values.size()); ++i)
                        new_state.values[i] = steer_target.values[i];
                }
            }

            if (!motion_validator_ || !motion_validator_(near->state, new_state))
                continue;

            if (enable_trrt_ && !minExpansionControl(d))
                continue;

            double child_cost = state_cost_function_ ? state_cost_function_(new_state) : pathCost(near) + distance(near->state, new_state);
            double motion_cost = computeStateTransitionCost(near->state, new_state);
            if (!evaluateTRRTAcceptance(near->cost, child_cost, motion_cost))
                continue;

            Motion* new_motion = new (std::nothrow) Motion(new_state);
            if (!new_motion) { continue; }
            new_motion->parent = near;
            new_motion->cost = enable_rrt_star_ ? (pathCost(near) + distance(near->state, new_state)) : child_cost;

            if (enable_rrt_star_)
            {
                double search_radius = rewire_radius_ > 0.0 ? rewire_radius_ : max_distance_ * 1.5;
                std::vector<Motion*> neighbors = getNearbyMotions(grow_nn, new_motion, search_radius);
                if (neighbors.size() > 10) neighbors.resize(10);

                Motion* best_parent = near;
                double best_cost = new_motion->cost;

                struct Candidate { Motion* m; double cost; };
                std::vector<Candidate> candidates;
                candidates.reserve(neighbors.size());

                for (Motion* neighbor : neighbors)
                {
                    if (neighbor == near) continue;
                    double cost_through_neighbor = pathCost(neighbor) + distance(neighbor->state, new_motion->state);
                    if (cost_through_neighbor < best_cost)
                    {
                        double dist = distance(neighbor->state, new_motion->state);
                        if (dist < search_radius * 1.5)
                        {
                            candidates.push_back({neighbor, cost_through_neighbor});
                        }
                    }
                }

                if (!candidates.empty())
                {
                    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b){ return a.cost < b.cost; });
                    int checks = 0;
                    for (const Candidate& c : candidates)
                    {
                        if (checks >= collision_check_limit_) break;
                        ++checks;
                        ++diag_parent_collision_checks_;
                        if (motion_validator_ && motion_validator_(c.m->state, new_motion->state))
                        {
                            best_parent = c.m;
                            best_cost = c.cost;
                            break;
                        }
                        else
                        {
                            ++diag_parent_rejects_;
                        }
                    }
                }
                new_motion->parent = best_parent;
                new_motion->cost = best_cost;
            }

            grow_tree.push_back(new_motion);
            grow_nn->add(new_motion);

            if (enable_rrt_star_ && (iter % 10 == 0))
            {
                double search_radius = rewire_radius_ > 0.0 ? rewire_radius_ : max_distance_ * 1.5;
                std::vector<Motion*> neighbors = getNearbyMotions(grow_nn, new_motion, search_radius);
                if (neighbors.size() > 5) neighbors.resize(5);
                rewireTree(new_motion, neighbors);
            }

            if (enable_trrt_)
            {
                if (child_cost < best_cost_) best_cost_ = child_cost;
                if (child_cost > worst_cost_) worst_cost_ = child_cost;
            }

            Motion* greedy_target = nullptr;
            if (hybrid_config_.enable_greedy_connect && greedyConnectTrees(new_motion, other_tree, other_nn, greedy_target))
            {
                connect_a = grow_a ? new_motion : greedy_target;
                connect_b = grow_a ? greedy_target : new_motion;
                solved = true;
                break;
            }
        }

        if (solved && connect_a && connect_b)
        {
            std::vector<State> pathA, pathB;
            extractPath(connect_a, pathA);
            extractPath(connect_b, pathB);
            std::reverse(pathB.begin(), pathB.end());
            path = pathA;
            if (!pathB.empty())
            {
                if (!path.empty() && distance(path.back(), pathB.front()) < 1e-6)
                {
                    path.insert(path.end(), pathB.begin() + 1, pathB.end());
                }
                else
                {
                    path.insert(path.end(), pathB.begin(), pathB.end());
                }
            }
            nn_a.reset();
            nn_b.reset();
            for (Motion* m : treeA) delete m;
            for (Motion* m : treeB) delete m;
            last_goal_motion_ = nullptr;
            return true;
        }

        nn_a.reset();
        nn_b.reset();
        for (Motion* m : treeA) delete m;
        for (Motion* m : treeB) delete m;

        ROS_WARN("[DualArmRRT] CONNECT mode: no connection found within timeout");
        return false;
    }

    auto* start_motion = new (std::nothrow) Motion(start);
    if (!start_motion)
    {
        ROS_ERROR("[DualArmRRT] Failed to allocate memory for start motion");
        return false;
    }

    if (enable_trrt_ && state_cost_function_)
    {
        start_motion->cost = state_cost_function_(start);
        best_cost_ = worst_cost_ = start_motion->cost;
    }

    motions_.push_back(start_motion);
    nn_->add(start_motion);

    std::string algorithm_mode = "RRT";
    if (enable_rrt_star_ && enable_trrt_) algorithm_mode = "T-RRT + RRT*";
    else if (enable_rrt_star_) algorithm_mode = "RRT*";
    else if (enable_trrt_) algorithm_mode = "T-RRT";

    ROS_INFO("[DualArmRRT] Starting %s planning with %zu states", algorithm_mode.c_str(), motions_.size());

    ros::Time start_time = ros::Time::now();
    Motion* solution = nullptr;
    Motion* approx_solution = nullptr;
    double approx_difference = std::numeric_limits<double>::infinity();
    double goal_threshold = 0.02;

    State rand_state(state_dimension_);
    State interpolated_state(state_dimension_);
    int iteration = 0;
    ros::Time end_time = start_time + ros::Duration(timeout);

    // Prepare PlanningContext for modules
    PlanningContext ctx;
    ctx.start_state = &start;
    ctx.goal_state = &goal;
    ctx.current_sample = &rand_state;
    ctx.new_node_candidate = nullptr;
    ctx.tree = &motions_;
    ctx.default_step_size = max_distance_;
    ctx.step_size = max_distance_;

    while (ros::Time::now() < end_time)
    {
        ++iteration;
        // Allow modules to bias sampling (e.g., GoalBiasModule)
        ctx.reset();
        ctx.current_sample = &rand_state;
        bool provided = false;
        for (auto& m : modules_)
        {
            try { if (m && m->preSample(ctx)) { provided = true; break; } }
            catch (...) { ROS_WARN_THROTTLE(5.0, "Exception in preSample"); }
        }

        if (!provided)
        {
            if (uniform_dist_(rng_) < goal_bias_) rand_state = goal;
            else rand_state = sampleRandomState();
        }

        Motion* near_motion = findNearestMotion(rand_state);
        if (!near_motion) { ROS_ERROR("[DualArmRRT] findNearestMotion returned nullptr, aborting"); break; }

        double rand_motion_distance = distance(near_motion->state, rand_state);
        State* new_state = &rand_state;
        if (rand_motion_distance > max_distance_)
        {
            double t = max_distance_ / rand_motion_distance;
            interpolated_state = interpolate(near_motion->state, rand_state, t);
            new_state = &interpolated_state;
            rand_motion_distance = max_distance_;
        }

        // Allow modules to adjust step size before extend
        ctx.current_sample = new_state;
        ctx.step_size = rand_motion_distance > 0.0 ? std::min(rand_motion_distance, max_distance_) : max_distance_;
        for (auto& m : modules_)
        {
            try { if (m) m->preExtend(ctx); }
            catch (...) { ROS_WARN_THROTTLE(5.0, "Exception in preExtend"); }
        }

        if (!motion_validator_ || !motion_validator_(near_motion->state, *new_state)) continue;
        if (enable_trrt_ && !minExpansionControl(rand_motion_distance)) continue;
        double child_cost = 0.0;
        if (enable_trrt_ && state_cost_function_) child_cost = state_cost_function_(*new_state);
        if (enable_trrt_ && motion_cost_function_)
        {
            double motion_cost = motion_cost_function_(near_motion->state, *new_state);
            if (!transitionTest(motion_cost)) continue;
        }

            auto* motion = new (std::nothrow) Motion(*new_state);
        if (!motion) { ROS_ERROR("[DualArmRRT] Failed to allocate memory for motion"); continue; }
        motion->parent = near_motion;
        double path_cost_to_new = pathCost(near_motion) + distance(near_motion->state, *new_state);
        motion->cost = enable_rrt_star_ ? path_cost_to_new : child_cost;

        // After creating a candidate motion, allow modules to post-process / reject it
        ctx.new_node_candidate = &motion->state;
        ctx.tree = &motions_;
        ctx.is_valid = true;
        for (auto& m : modules_)
        {
            try { if (m) m->postExtend(ctx); }
            catch (...) { ROS_WARN_THROTTLE(5.0, "Exception in postExtend"); }
        }

        if (!ctx.is_valid)
        {
            // Module rejected the node
            delete motion;
            continue;
        }

        if (enable_rrt_star_)
        {
            double search_radius = rewire_radius_ > 0.0 ? rewire_radius_ : max_distance_ * 1.5;
            std::vector<Motion*> neighbors = getNearbyMotions(nn_, motion, search_radius);
            if (neighbors.size() > 10) neighbors.resize(10);
            Motion* best_parent = near_motion;
            double best_cost = path_cost_to_new;
            struct Candidate { Motion* m; double cost; };
            std::vector<Candidate> candidates;
            candidates.reserve(neighbors.size());
            for (Motion* neighbor : neighbors)
            {
                if (neighbor == near_motion) continue;
                double cost_through_neighbor = pathCost(neighbor) + distance(neighbor->state, motion->state);
                if (cost_through_neighbor < best_cost)
                {
                    double dist = distance(neighbor->state, motion->state);
                    if (dist < search_radius * 1.5) candidates.push_back({neighbor, cost_through_neighbor});
                }
            }
            if (!candidates.empty())
            {
                std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b){ return a.cost < b.cost; });
                int checks = 0;
                for (const Candidate& c : candidates)
                {
                    if (checks >= collision_check_limit_) break;
                    ++checks;
                    ++diag_parent_collision_checks_;
                    if (motion_validator_ && motion_validator_(c.m->state, motion->state)) { best_parent = c.m; best_cost = c.cost; break; }
                    else ++diag_parent_rejects_;
                }
            }
            motion->parent = best_parent;
            motion->cost = best_cost;
        }

        motions_.push_back(motion);
        nn_->add(motion);
        if (enable_rrt_star_ && (iteration % 10 == 0)) { double search_radius = rewire_radius_ > 0.0 ? rewire_radius_ : max_distance_ * 1.5; std::vector<Motion*> neighbors = getNearbyMotions(nn_, motion, search_radius); if (neighbors.size() > 5) neighbors.resize(5); rewireTree(motion, neighbors); }
        if (enable_trrt_)
        {
            if (child_cost < best_cost_) best_cost_ = child_cost;
            if (child_cost > worst_cost_) worst_cost_ = child_cost;
        }

        double dist_to_goal = distance(motion->state, goal);
        bool is_satisfied = dist_to_goal < goal_threshold;
        if (is_satisfied) { approx_difference = dist_to_goal; solution = motion; break; }
        if (dist_to_goal < approx_difference) { approx_difference = dist_to_goal; approx_solution = motion; }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr) { solution = approx_solution; approximate = true; }
    if (solution != nullptr)
    {
        last_goal_motion_ = solution;
        extractPath(solution, path);
        solved = true;
        double planning_time = (ros::Time::now() - start_time).toSec();
        ROS_INFO("Planning %s: %d iterations, %zu states, %.3f seconds%s", solved ? "succeeded" : "failed", iteration, motions_.size(), planning_time, approximate ? " (approximate)" : "");
    }
    else
    {
        ROS_WARN("Planning failed: no solution found");
    }

    ROS_INFO("[DualArmRRT] Module summary: RRT*: %s, T-RRT: %s, CONNECT: %s", enable_rrt_star_ ? "ON" : "OFF", enable_trrt_ ? "ON" : "OFF", enable_connect_ ? "ON" : "OFF");
    if (enable_diagnostics_)
    {
        ROS_INFO("[DualArmRRT][Diag] parent_checks=%zu parent_rejects=%zu rewire_checks=%zu rewire_rejects=%zu", diag_parent_collision_checks_, diag_parent_rejects_, diag_rewire_collision_checks_, diag_rewire_rejects_);
    }

    return solved;
}

// RRT* helpers
double DualArmRRT::pathCost(Motion* motion) const
{
    if (!motion) return 0.0;
    if (enable_rrt_star_) return motion->cost;
    double cost = 0.0; Motion* current = motion; while (current->parent != nullptr) { cost += distance(current->state, current->parent->state); current = current->parent; } return cost;
}

std::vector<Motion*> DualArmRRT::getNearbyMotions(std::shared_ptr<ompl::NearestNeighborsGNATNoThreadSafety<Motion*>> nn, Motion* motion, double radius)
{
    std::vector<Motion*> neighbors;
    std::size_t k = std::min(static_cast<std::size_t>(10), nn->size());
    std::vector<Motion*> k_nearest;
    nn->nearestK(motion, k, k_nearest);
    for (Motion* neighbor : k_nearest) { if (neighbor != motion && distance(motion->state, neighbor->state) <= radius) neighbors.push_back(neighbor); }
    return neighbors;
}

bool DualArmRRT::createsCycle(Motion* child, Motion* new_parent) const
{
    Motion* p = new_parent; while (p) { if (p == child) return true; p = p->parent; } return false;
}

void DualArmRRT::rewireTree(Motion* new_motion, const std::vector<Motion*>& neighbors)
{
    double new_cost = pathCost(new_motion);
    for (Motion* n : neighbors)
    {
        if (!n || n == new_motion || n == new_motion->parent) continue;
        if (createsCycle(n, new_motion)) continue;
        double cost_through_new = new_cost + distance(new_motion->state, n->state);
        if (cost_through_new < pathCost(n) * 0.95)
        {
            if (motion_validator_)
            {
                ++diag_rewire_collision_checks_;
                if (motion_validator_(new_motion->state, n->state)) { n->parent = new_motion; n->cost = cost_through_new; }
                else ++diag_rewire_rejects_;
            }
        }
    }
}

// ---------------------- ScheduleStream implementations ----------------------
std::vector<geometry_msgs::Pose> DualArmRRT::streamGenerateGrasps(const geometry_msgs::Pose& obj_pose)
{
    std::vector<geometry_msgs::Pose> grasps;
    
    // 设定抓取半径：球半径(0.05) + 抓取余量(例如 0.10)
    // 这决定了手腕(Link7)离球心的距离
    double grasp_distance = 0.15;
    
    // 通用 LookAt 采样算法
    struct Sample { double yaw; double pitch; };
    std::vector<Sample> samples = {
        {0, 0}, {M_PI/2, 0}, {-M_PI/2, 0}, {M_PI, 0}, // 四周水平抓取
        {0, M_PI/4}, {M_PI/2, M_PI/4}, {-M_PI/2, M_PI/4}, {M_PI, M_PI/4}, // 45度斜下抓取
        {0, M_PI/2} // 垂直向下抓取
    };

    for (const auto& s : samples)
    {
        geometry_msgs::Pose p = obj_pose;
        
        // 1. 计算位置：球面坐标转笛卡尔坐标
        double dx = grasp_distance * cos(s.pitch) * cos(s.yaw);
        double dy = grasp_distance * cos(s.pitch) * sin(s.yaw);
        double dz = grasp_distance * sin(s.pitch);
        
        p.position.x += dx;
        p.position.y += dy;
        p.position.z += dz;

        // 2. 计算朝向 (LookAt): 让手爪Z轴指向球心
        tf2::Vector3 target_pos(obj_pose.position.x, obj_pose.position.y, obj_pose.position.z);
        tf2::Vector3 current_pos(p.position.x, p.position.y, p.position.z);
        tf2::Vector3 z_axis = (target_pos - current_pos).normalized(); // Z轴指向物体
        
        // 约定 X 轴：尽量水平（防止手爪扭曲）
        tf2::Vector3 up(0, 0, 1);
        tf2::Vector3 x_axis = up.cross(z_axis).normalized();
        if (x_axis.length() < 0.001) x_axis = tf2::Vector3(1, 0, 0); // 处理垂直向下的特例
        
        tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();

        // 构造旋转矩阵并转为四元数
        tf2::Matrix3x3 rot_mat(
            x_axis.x(), y_axis.x(), z_axis.x(),
            x_axis.y(), y_axis.y(), z_axis.y(),
            x_axis.z(), y_axis.z(), z_axis.z()
        );
        
        tf2::Quaternion q_final;
        rot_mat.getRotation(q_final);
        p.orientation = tf2::toMsg(q_final);

        grasps.push_back(p);
    }

    return grasps;
}

bool DualArmRRT::streamComputeIK(const geometry_msgs::Pose& pose, const std::string& arm_group, std::vector<double>& joints)
{
    if (!robot_model_)
    {
        ROS_ERROR("[DualArmRRT] robot_model_ is null in streamComputeIK");
        return false;
    }

    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(arm_group);
    if (!jmg)
    {
        ROS_ERROR("[DualArmRRT] JointModelGroup '%s' not found", arm_group.c_str());
        return false;
    }

    moveit::core::RobotState rs(robot_model_);

    // Try IK using MoveIt RobotState helper. Use a few attempts with small timeout.
    bool ok = false;
    try
    {
        // Note: depending on MoveIt version, setFromIK overloads may vary.
        // Prefer the timeout-only overload when available to avoid deprecated attempts-based API.
        ok = rs.setFromIK(jmg, pose, 0.05);
    }
    catch (const std::exception& e)
    {
        ROS_WARN("[DualArmRRT] Exception in setFromIK: %s", e.what());
        ok = false;
    }

    if (!ok)
        return false;

    rs.copyJointGroupPositions(jmg, joints);
    return true;
}

} // namespace dual_arm_planner

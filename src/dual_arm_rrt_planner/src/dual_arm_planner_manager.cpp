#include "dual_arm_rrt_planner/dual_arm_planner_manager.h"
#include "dual_arm_rrt_planner/dual_arm_rrt.h"
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/Pose.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <pluginlib/class_list_macros.hpp>

namespace dual_arm_planner
{

// Planning Context实现 (adapted from ImprovedRRTPlanningContext)
class DualArmPlanningContext : public planning_interface::PlanningContext
{
public:
    DualArmPlanningContext(const std::string& name,
                               const std::string& group,
                               const moveit::core::RobotModelConstPtr& model)
        : planning_interface::PlanningContext(name, group)
        , robot_model_(model)
        , range_(0.0)
        , goal_bias_(0.05)
        , enable_trrt_(false)
        , init_temperature_(100.0)
        , temp_change_factor_(0.1)
        , cost_threshold_(std::numeric_limits<double>::infinity())
        , frontier_threshold_(0.0)
        , frontier_node_ratio_(0.1)
        , enable_rrt_star_(false)
        , rewire_radius_(0.0)
        , enable_connect_(false)
    {
        ros::NodeHandle nh("~");
        ROS_INFO("[DualArmPlanningContext] Created for group '%s'", group.c_str());
        // Log module enablement (check for module-related params in the node's private namespace)
        {
            double tmp_d;
            bool tmp_b;
            bool goalbias = false, variablestep = false, apf = false, nodereject = false, trrt = false;
            if (nh.getParam("goal_bias", tmp_d)) goalbias = (tmp_d > 0.0);
            if (nh.getParam("variable_min_step", tmp_d)) variablestep = true;
            if (nh.getParam("apf_attractive_gain", tmp_d) || nh.getParam("apf_use_repulsive_force", tmp_b)) apf = true;
            if (nh.getParam("node_rejection_radius_factor", tmp_d)) nodereject = true;
            if (nh.getParam("enable_trrt", tmp_b) || nh.getParam("t_rrt_init_temperature", tmp_d)) trrt = true;
            ROS_INFO("[DualArmPlanningContext] Modules enabled -> GoalBias: %s, VariableStep: %s, APF: %s, NodeRejection: %s, TRRT: %s",
                     goalbias ? "true" : "false",
                     variablestep ? "true" : "false",
                     apf ? "true" : "false",
                     nodereject ? "true" : "false",
                     trrt ? "true" : "false");
        }
    }
    
    virtual ~DualArmPlanningContext() {}
    
    virtual bool solve(planning_interface::MotionPlanResponse& res) override
    {
        return solve(res, planning_interface::MotionPlanDetailedResponse());
    }
    
    virtual bool solve(planning_interface::MotionPlanDetailedResponse& res) override
    {
        planning_interface::MotionPlanResponse simple_res;
        bool success = solve(simple_res, res);
        res.error_code_ = simple_res.error_code_;
        if (success)
        {
            res.trajectory_.push_back(simple_res.trajectory_);
            res.processing_time_.push_back(simple_res.planning_time_);
            res.description_.push_back("DualArmRRT");
        }
        return success;
    }
    
    virtual bool solve(planning_interface::MotionPlanResponse& res,
                      const planning_interface::MotionPlanDetailedResponse& /*detailed_res*/)
    {
        ros::Time start_time = ros::Time::now();
        const moveit::core::JointModelGroup* joint_model_group = 
            robot_model_->getJointModelGroup(getGroupName());
        if (!joint_model_group)
        {
            ROS_ERROR("Joint model group '%s' not found", getGroupName().c_str());
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            return false;
        }

        moveit::core::RobotState start_state = getPlanningScene()->getCurrentState();

        std::vector<double> start_joint_values;
        std::vector<double> goal_joint_values;
        start_state.copyJointGroupPositions(joint_model_group, start_joint_values);
        goal_joint_values.resize(start_joint_values.size());

        // Final assembled 14-dim goal vector
        std::vector<double> final_goal_joint_values;

        // Case A: Joint constraints provided
        if (!request_.goal_constraints.empty() && !request_.goal_constraints[0].joint_constraints.empty())
        {
            const auto& joint_constraints = request_.goal_constraints[0].joint_constraints;
            for (const auto& jc : joint_constraints)
            {
                const moveit::core::JointModel* jm = joint_model_group->getJointModel(jc.joint_name);
                if (jm)
                {
                    int idx = joint_model_group->getVariableGroupIndex(jc.joint_name);
                    if (idx >= 0 && idx < static_cast<int>(goal_joint_values.size()))
                    {
                        goal_joint_values[idx] = jc.position;
                    }
                }
            }
            final_goal_joint_values = goal_joint_values;
        }
        else if (!request_.goal_constraints.empty() && !request_.goal_constraints[0].position_constraints.empty())
        {
            // Pose goal: use Streams to infer a joint goal
            ROS_INFO("[DualArm] Detected Pose Goal, inferring joint state via Streams...");

            DualArmRRT temp_rrt;
            // Provide robot model for IK
            temp_rrt.setPlanningScene(getPlanningScene(), robot_model_, getGroupName());

            const auto& p_const = request_.goal_constraints[0].position_constraints[0];
            geometry_msgs::Pose target_pose;
            if (!p_const.constraint_region.primitive_poses.empty())
                target_pose.position = p_const.constraint_region.primitive_poses[0].position;
            else
            {
                ROS_ERROR("Position constraint has no primitive poses");
                res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
                return false;
            }

            if (!request_.goal_constraints[0].orientation_constraints.empty())
                target_pose.orientation = request_.goal_constraints[0].orientation_constraints[0].orientation;
            else
            {
                // If no orientation provided, use identity
                target_pose.orientation.w = 1.0;
                target_pose.orientation.x = 0.0;
                target_pose.orientation.y = 0.0;
                target_pose.orientation.z = 0.0;
            }

            std::string target_link = p_const.link_name;
            std::string active_arm_group = "";
            if (target_link.find("left") != std::string::npos) active_arm_group = "left_arm";
            else if (target_link.find("right") != std::string::npos) active_arm_group = "right_arm";

            if (active_arm_group.empty())
            {
                ROS_ERROR("Could not infer arm group from link name: %s", target_link.c_str());
                return false;
            }

            auto grasps = temp_rrt.streamGenerateGrasps(target_pose);

            const moveit::core::JointModelGroup* arm_jmg = robot_model_->getJointModelGroup(active_arm_group);
            if (!arm_jmg)
            {
                ROS_ERROR("Arm JointModelGroup '%s' not found", active_arm_group.c_str());
                res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
                return false;
            }

            bool found_safe_goal = false;
            std::vector<double> arm_solution;

            // Try multiple grasps + IKs and reject any candidate whose assembled full-body
            // joint state is in collision. This is goal-rejection sampling.
            for (const auto& grasp : grasps)
            {
                if (!temp_rrt.streamComputeIK(grasp, active_arm_group, arm_solution))
                    continue; // no IK for this grasp, try next

                // Build a candidate full-body RobotState from start + this arm IK
                moveit::core::RobotState candidate_state(robot_model_);
                candidate_state.setJointGroupPositions(joint_model_group, start_joint_values);
                candidate_state.setJointGroupPositions(arm_jmg, arm_solution);
                candidate_state.update();

                auto planning_scene_ptr = getPlanningScene();
                if (!planning_scene_ptr)
                {
                    ROS_ERROR("PlanningScene is null while checking candidate goal");
                    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
                    return false;
                }

                // If this candidate full-state is not in collision, accept it as the goal
                if (!planning_scene_ptr->isStateColliding(candidate_state, getGroupName()))
                {
                    found_safe_goal = true;
                    // Copy out full joint vector for the joint_model_group
                    final_goal_joint_values.resize(start_joint_values.size());
                    candidate_state.copyJointGroupPositions(joint_model_group, final_goal_joint_values);
                    break;
                }
                // otherwise reject and continue sampling
            }

            if (!found_safe_goal)
            {
                ROS_WARN("No collision-free goal found from sampled grasps/IKs");
                res.error_code_.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
                return false;
            }
        }
        else
        {
            ROS_ERROR("No valid constraints found");
            return false;
        }

        // 获取关节限制
        std::vector<double> lower_limits, upper_limits;
        const std::vector<const moveit::core::JointModel*>& joint_models = 
            joint_model_group->getActiveJointModels();
        for (const auto* jm : joint_models)
        {
            const moveit::core::VariableBounds& bounds = jm->getVariableBounds()[0];
            lower_limits.push_back(bounds.min_position_);
            upper_limits.push_back(bounds.max_position_);
        }

        // 创建DualArmRRT实例
        DualArmRRT rrt;
        rrt.setStateDimension(start_joint_values.size());
        rrt.setJointLimits(lower_limits, upper_limits);
        rrt.setRange(0.5);
        rrt.setGoalBias(0.05);
        rrt.setPlanningScene(getPlanningScene(), robot_model_, getGroupName());

        // 状态验证器
        auto planning_scene = getPlanningScene();
        std::string group_name = getGroupName();
    rrt.setStateValidityChecker([planning_scene, group_name, joint_model_group, robot_model = robot_model_](const State& state) -> bool {
            try
            {
                if (!planning_scene)
                {
                    ROS_ERROR("[DualArmRRT] PlanningScene is null in state validity checker");
                    return false;
                }
                moveit::core::RobotState scratch_state(robot_model);
                scratch_state.setJointGroupPositions(joint_model_group, state.values);
                scratch_state.update();
                return !planning_scene->isStateColliding(scratch_state, group_name);
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("[DualArmRRT] Exception in state validity checker: %s", e.what());
                return false;
            }
        });

        // 【新增修复】设置运动验证器 (检查路径中间是否穿过障碍物)
        // 必须设置这个，否则 DualArmRRT 会报 "No motion validator set" 错误
        rrt.setMotionValidator([planning_scene, group_name, joint_model_group, robot_model = robot_model_](const State& from, const State& to) -> bool {
            // 在 Lambda 内部创建临时的 RobotState 以保证线程安全
            moveit::core::RobotState intermediate_state(robot_model);
            
            try
            {
                if (!planning_scene) return false;

                // 1. 计算两点间的关节空间距离
                double dist = 0.0;
                for (size_t i = 0; i < from.values.size(); ++i) {
                    double d = to.values[i] - from.values[i];
                    dist += d*d;
                }
                dist = std::sqrt(dist);

                // 2. 决定插值步数 (防止穿墙)
                // 分辨率设为 0.05弧度 (约3度)，如果距离很远，步数就多
                const double resolution = 0.05; 
                int steps = std::max(2, static_cast<int>(dist / resolution));

                // 3. 逐步检查中间状态
                std::vector<double> intermediate_values(from.values.size());
                for (int i = 1; i < steps; ++i)
                {
                    double t = static_cast<double>(i) / steps;
                    
                    // 线性插值
                    for (size_t j = 0; j < from.values.size(); ++j) {
                        intermediate_values[j] = from.values[j] + t * (to.values[j] - from.values[j]);
                    }
                    
                    // 更新机器人状态并检查碰撞
                    intermediate_state.setJointGroupPositions(joint_model_group, intermediate_values);
                    intermediate_state.update();

                    if (planning_scene->isStateColliding(intermediate_state, group_name))
                    {
                        return false; // 中间碰到障碍物了，路径无效
                    }
                }
                
                return true; // 全程无碰撞
            }
            catch (...) 
            { 
                ROS_ERROR("Exception in motion validator");
                return false; 
            }
        });

        // 执行规划（简化示例）
    State start_rrt(start_joint_values);
    State goal_rrt(final_goal_joint_values);
        std::vector<State> path;
        double timeout = request_.allowed_planning_time;
        if (timeout <= 0.0) timeout = 5.0;

        bool success = rrt.solve(start_rrt, goal_rrt, path, timeout);

        if (success && !path.empty())
        {
            robot_trajectory::RobotTrajectoryPtr trajectory(
                new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));
            moveit::core::RobotState reference_state = getPlanningScene()->getCurrentState();
            for (size_t i = 0; i < path.size(); ++i)
            {
                const auto& state = path[i];
                moveit::core::RobotState robot_state(reference_state);
                robot_state.setJointGroupPositions(joint_model_group, state.values);
                robot_state.update();
                if (!robot_state.satisfiesBounds(joint_model_group))
                    robot_state.enforceBounds(joint_model_group);
                trajectory->addSuffixWayPoint(robot_state, 0.0);
            }

            trajectory->unwind();
            res.trajectory_ = trajectory;
            res.planning_time_ = (ros::Time::now() - start_time).toSec();
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
            return true;
        }
        else
        {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }
    }
    
    virtual bool terminate() override { return true; }
    virtual void clear() override {}

private:
    moveit::core::RobotModelConstPtr robot_model_;
    double range_;
    double goal_bias_;
    bool enable_trrt_;
    double init_temperature_;
    double temp_change_factor_;
    double cost_threshold_;
    double frontier_threshold_;
    double frontier_node_ratio_;
    bool enable_rrt_star_;
    double rewire_radius_;
    bool enable_connect_;
};

// PlannerManager实现
DualArmPlannerManager::DualArmPlannerManager()
    : nh_("~")
{
}

bool DualArmPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                          const std::string& ns)
{
    robot_model_ = model;
    return true;
}

void DualArmPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
    algs.clear();
    algs.push_back("DualArmRRT");
}

planning_interface::PlanningContextPtr DualArmPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    ROS_INFO("[DualArmPlannerManager] getPlanningContext() ENTER for group: %s", req.group_name.c_str());
    if (req.group_name.empty())
    {
        ROS_ERROR("[DualArmPlannerManager] No group specified for planning");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return planning_interface::PlanningContextPtr();
    }

    planning_interface::PlanningContextPtr context(
        new DualArmPlanningContext("DualArmRRT", req.group_name, robot_model_));
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return context;
}

bool DualArmPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    return !req.group_name.empty();
}

} // namespace dual_arm_planner

// 注册插件
PLUGINLIB_EXPORT_CLASS(dual_arm_planner::DualArmPlannerManager,
                       planning_interface::PlannerManager)

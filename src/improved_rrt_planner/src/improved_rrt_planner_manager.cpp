#include "improved_rrt_planner/improved_rrt_planner_manager.h"
#include "improved_rrt_planner/improved_rrt.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <pluginlib/class_list_macros.hpp>

namespace improved_rrt_planner
{

// Planning Context实现
class ImprovedRRTPlanningContext : public planning_interface::PlanningContext
{
public:
    ImprovedRRTPlanningContext(const std::string& name,
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
        // 从参数服务器读取RRT参数（使用合理的默认值）
        ros::NodeHandle nh("~");
        
        // 尝试从多个可能的命名空间读取参数
        if (!nh.getParam("planner_configs/ImprovedRRT/range", range_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/range", range_))
            {
                ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
                if (!nh_pipeline.getParam("planner_configs/ImprovedRRT/range", range_))
                {
                    range_ = 0.5;  // 使用合理的默认值
                    ROS_WARN("[ImprovedRRTPlanningContext] Could not read 'range' parameter, using default: %.3f", range_);
                }
            }
        }
        
        if (!nh.getParam("planner_configs/ImprovedRRT/goal_bias", goal_bias_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/goal_bias", goal_bias_))
            {
                ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
                if (!nh_pipeline.getParam("planner_configs/ImprovedRRT/goal_bias", goal_bias_))
                {
                    goal_bias_ = 0.05;  // 使用默认值
                }
            }
        }
        
        // Read T-RRT parameters with defaults (try private, then /move_group, then pipeline)
        if (!nh.getParam("planner_configs/ImprovedRRT/enable_trrt", enable_trrt_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/enable_trrt", enable_trrt_))
                nh_pipeline.param("planner_configs/ImprovedRRT/enable_trrt", enable_trrt_, false);
        }

        if (!nh.getParam("planner_configs/ImprovedRRT/init_temperature", init_temperature_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/init_temperature", init_temperature_))
                nh_pipeline.param("planner_configs/ImprovedRRT/init_temperature", init_temperature_, 100.0);
        }

        if (!nh.getParam("planner_configs/ImprovedRRT/temp_change_factor", temp_change_factor_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/temp_change_factor", temp_change_factor_))
                nh_pipeline.param("planner_configs/ImprovedRRT/temp_change_factor", temp_change_factor_, 0.1);
        }

        if (!nh.getParam("planner_configs/ImprovedRRT/cost_threshold", cost_threshold_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/cost_threshold", cost_threshold_))
                nh_pipeline.param("planner_configs/ImprovedRRT/cost_threshold", cost_threshold_, std::numeric_limits<double>::infinity());
        }

        if (!nh.getParam("planner_configs/ImprovedRRT/frontier_threshold", frontier_threshold_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/frontier_threshold", frontier_threshold_))
                nh_pipeline.param("planner_configs/ImprovedRRT/frontier_threshold", frontier_threshold_, 0.0);
        }

        if (!nh.getParam("planner_configs/ImprovedRRT/frontier_node_ratio", frontier_node_ratio_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/frontier_node_ratio", frontier_node_ratio_))
                nh_pipeline.param("planner_configs/ImprovedRRT/frontier_node_ratio", frontier_node_ratio_, 0.1);
        }
        
        // Read RRT* parameters with defaults (try private, then /move_group, then pipeline)
        if (!nh.getParam("planner_configs/ImprovedRRT/enable_rrt_star", enable_rrt_star_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/enable_rrt_star", enable_rrt_star_))
                nh_pipeline.param("planner_configs/ImprovedRRT/enable_rrt_star", enable_rrt_star_, false);
        }

        if (!nh.getParam("planner_configs/ImprovedRRT/rewire_radius", rewire_radius_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/rewire_radius", rewire_radius_))
                nh_pipeline.param("planner_configs/ImprovedRRT/rewire_radius", rewire_radius_, 0.0);
        }

        // Read Connect parameter (enable/disable bidirectional connect)
        if (!nh.getParam("planner_configs/ImprovedRRT/enable_connect", enable_connect_))
        {
            ros::NodeHandle nh_move_group("/move_group");
            ros::NodeHandle nh_pipeline("/move_group/planning_pipelines/improved_rrt");
            if (!nh_move_group.getParam("planner_configs/ImprovedRRT/enable_connect", enable_connect_))
                nh_pipeline.param("planner_configs/ImprovedRRT/enable_connect", enable_connect_, false);
        }
        
        // 输出配置信息
        std::string mode = "RRT";
        if (enable_rrt_star_ && enable_trrt_) mode = "T-RRT+RRT*";
        else if (enable_rrt_star_) mode = "RRT*";
        else if (enable_trrt_) mode = "T-RRT";
        
    ROS_INFO("[ImprovedRRT] Initialized %s planner for group '%s'", mode.c_str(), group.c_str());
    ROS_INFO("[ImprovedRRTPlanningContext] Params read: enable_trrt=%s, enable_rrt_star=%s, enable_connect=%s, range=%.3f, rewire_radius=%.3f", 
         enable_trrt_ ? "true" : "false",
         enable_rrt_star_ ? "true" : "false",
         enable_connect_ ? "true" : "false",
         range_, rewire_radius_);
    }
    
    virtual ~ImprovedRRTPlanningContext() {
    }
    
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
            res.description_.push_back("ImprovedRRT");
        }
        return success;
    }
    
    virtual bool solve(planning_interface::MotionPlanResponse& res,
                      const planning_interface::MotionPlanDetailedResponse& /*detailed_res*/)
    {
        ros::Time start_time = ros::Time::now();
        
        // 获取规划组
        const moveit::core::JointModelGroup* joint_model_group = 
            robot_model_->getJointModelGroup(getGroupName());
        
        if (!joint_model_group)
        {
            ROS_ERROR("Joint model group '%s' not found", getGroupName().c_str());
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
            return false;
        }
        
        // 获取起始状态 - 使用当前状态而不是请求中的状态
        moveit::core::RobotState start_state = getPlanningScene()->getCurrentState();
        
        // 获取目标约束
        if (request_.goal_constraints.empty() || 
            request_.goal_constraints[0].joint_constraints.empty())
        {
            ROS_ERROR("No joint goal constraints specified");
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
            return false;
        }
        
        // 提取起始和目标关节值
        std::vector<double> start_joint_values;
        std::vector<double> goal_joint_values;
        std::vector<std::string> joint_names;
        
        start_state.copyJointGroupPositions(joint_model_group, start_joint_values);
        
        // 从目标约束中提取关节值
        const auto& joint_constraints = request_.goal_constraints[0].joint_constraints;
        goal_joint_values.resize(start_joint_values.size());
        
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
        
        // 创建RRT算法实例
        ImprovedRRT rrt;
        rrt.setStateDimension(start_joint_values.size());
        rrt.setJointLimits(lower_limits, upper_limits);
        
        double actual_range = (range_ > 0.0) ? range_ : 0.5;
        rrt.setRange(actual_range);
        rrt.setGoalBias(goal_bias_);
        
        // T-RRT parameters (optional)
        rrt.setEnableTRRT(enable_trrt_);
        if (enable_trrt_)
        {
            rrt.setInitTemperature(init_temperature_);
            rrt.setTempChangeFactor(temp_change_factor_);
            rrt.setCostThreshold(cost_threshold_);
            rrt.setFrontierThreshold(frontier_threshold_);
            rrt.setFrontierNodeRatio(frontier_node_ratio_);
        }
        
        // RRT* parameters (optional)
        rrt.setEnableRRTStar(enable_rrt_star_);
        if (enable_rrt_star_)
        {
            rrt.setRewireRadius(rewire_radius_);
        }
        // Connect parameter (optional)
        rrt.setEnableConnect(enable_connect_);
        
        // 设置 PlanningScene 用于代价计算
        rrt.setPlanningScene(getPlanningScene(), robot_model_, getGroupName());
        
        // 设置状态验证器（使用MoveIt的碰撞检测）
        // Use init-capture to create a RobotState that lives with the lambda
        rrt.setStateValidityChecker([this, joint_model_group, scratch_state = moveit::core::RobotState(robot_model_)](const State& state) mutable -> bool {
            try
            {
                if (!getPlanningScene())
                {
                    ROS_ERROR("[ImprovedRRT] PlanningScene is null in state validity checker");
                    return false;
                }
                scratch_state.setJointGroupPositions(joint_model_group, state.values);
                scratch_state.update();
                return !getPlanningScene()->isStateColliding(scratch_state, getGroupName());
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("[ImprovedRRT] Exception in state validity checker: %s", e.what());
                return false;
            }
        });
        
        // 设置运动验证器（模仿OMPL的checkMotion行为）
        // Use init-capture for scratch states
        // Reverted to discrete checks because CCD is not implemented in current setup
        rrt.setMotionValidator([this, joint_model_group, 
                              state_from = moveit::core::RobotState(robot_model_), 
                              state_to = moveit::core::RobotState(robot_model_), 
                              intermediate_state = moveit::core::RobotState(robot_model_)](const State& from, const State& to) mutable -> bool {
            try
            {
                if (!getPlanningScene())
                {
                    ROS_ERROR("[ImprovedRRT] PlanningScene is null in motion validator");
                    return false;
                }

                state_from.setJointGroupPositions(joint_model_group, from.values);
                state_to.setJointGroupPositions(joint_model_group, to.values);
                state_from.update();
                state_to.update();

                // Check end state first
                if (getPlanningScene()->isStateColliding(state_to, getGroupName()))
                    return false;

                // Calculate distance
                double distance = 0.0;
                for (size_t j = 0; j < from.values.size(); ++j)
                {
                    double diff = to.values[j] - from.values[j];
                    distance += diff * diff;
                }
                distance = std::sqrt(distance);
                
                // Adaptive checking steps
                const double resolution = 0.01; // High precision (approx 0.5 deg) to prevent rubbing
                int num_steps = std::max(2, static_cast<int>(std::ceil(distance / resolution)));
                // Removed step cap to ensure safety for long motions
                
                std::vector<double> intermediate_values(from.values.size());
                for (int i = 1; i < num_steps; ++i)
                {
                    double t = static_cast<double>(i) / num_steps;
                    
                    for (size_t j = 0; j < from.values.size(); ++j)
                    {
                        intermediate_values[j] = from.values[j] + t * (to.values[j] - from.values[j]);
                    }
                    
                    intermediate_state.setJointGroupPositions(joint_model_group, intermediate_values);
                    intermediate_state.update();
                    
                    if (getPlanningScene()->isStateColliding(intermediate_state, getGroupName()))
                        return false;
                }
                
                return true;
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("[ImprovedRRT] Exception in motion validator (checkMotion): %s", e.what());
                return false;
            }
        });

        /*
        // 原始实现（回退用）
        rrt.setMotionValidator([this, joint_model_group](const State& from, const State& to) -> bool {
            try
            {
                if (!getPlanningScene())
                {
                    ROS_ERROR("[ImprovedRRT] PlanningScene is null in motion validator");
                    return false;
                }
                
                moveit::core::RobotState state1(robot_model_);
                moveit::core::RobotState state2(robot_model_);
                state1.setJointGroupPositions(joint_model_group, from.values);
                state2.setJointGroupPositions(joint_model_group, to.values);
                state1.update();
                state2.update();
                
                // 检查终点状态（起点已经在树中，应该是有效的）
                if (getPlanningScene()->isStateColliding(state2, getGroupName()))
                    return false;
                
                // 计算关节空间的实际距离（类似OMPL的做法）
                double distance = 0.0;
                for (size_t j = 0; j < from.values.size(); ++j)
                {
                    double diff = to.values[j] - from.values[j];
                    distance += diff * diff;
                }
                distance = std::sqrt(distance);
                
                // 根据距离自适应确定检查步数
                const double resolution = 0.05; // 弧度
                int num_steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
                num_steps = std::min(num_steps, 50);
                
                for (int i = 1; i <= num_steps; ++i)
                {
                    double t = static_cast<double>(i) / (num_steps + 1);
                    moveit::core::RobotState intermediate_state(robot_model_);
                    std::vector<double> intermediate_values(from.values.size());
                    for (size_t j = 0; j < from.values.size(); ++j)
                    {
                        intermediate_values[j] = from.values[j] + t * (to.values[j] - from.values[j]);
                    }
                    intermediate_state.setJointGroupPositions(joint_model_group, intermediate_values);
                    intermediate_state.update();
                    if (getPlanningScene()->isStateColliding(intermediate_state, getGroupName()))
                        return false;
                }
                
                return true;
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("[ImprovedRRT] Exception in motion validator: %s", e.what());
                return false;
            }
        });
        */
        
        // 设置T-RRT代价函数（如果启用）
        if (enable_trrt_)
        {
            // 状态代价函数：使用简化的代价计算（避免distanceToCollision的线程安全问题）
            rrt.setStateCostFunction([this, joint_model_group, lower_limits, upper_limits](const State& state) -> double {
                moveit::core::RobotState robot_state(robot_model_);
                robot_state.setJointGroupPositions(joint_model_group, state.values);
                robot_state.update();
                
                // 简化的代价函数：使用关节位置的变化作为代价
                // 避免使用distanceToCollision，因为它可能有线程安全问题
                double cost = 0.0;
                const std::vector<double>& values = state.values;
                for (size_t i = 0; i < values.size(); ++i)
                {
                    // 计算与中间位置的偏差
                    double mid = (lower_limits[i] + upper_limits[i]) / 2.0;
                    double deviation = std::abs(values[i] - mid);
                    cost += deviation;
                }
                return cost;
            });
            
            // 运动代价函数：两个状态之间的代价差
            rrt.setMotionCostFunction([this, joint_model_group, lower_limits, upper_limits](const State& from, const State& to) -> double {
                moveit::core::RobotState state1(robot_model_);
                moveit::core::RobotState state2(robot_model_);
                state1.setJointGroupPositions(joint_model_group, from.values);
                state2.setJointGroupPositions(joint_model_group, to.values);
                state1.update();
                state2.update();
                
                // 简化的代价差计算
                double cost1 = 0.0, cost2 = 0.0;
                for (size_t i = 0; i < from.values.size(); ++i)
                {
                    double mid = (lower_limits[i] + upper_limits[i]) / 2.0;
                    cost1 += std::abs(from.values[i] - mid);
                    cost2 += std::abs(to.values[i] - mid);
                }
                
                return cost2 - cost1;  // 代价变化
            });
        }
        
        // 执行规划
        State start_rrt(start_joint_values);
        State goal_rrt(goal_joint_values);
        std::vector<State> path;
        
        // 使用MoveIt请求的超时时间
        double timeout = request_.allowed_planning_time;
        if (timeout <= 0.0) timeout = 5.0;
        
        // 开始规划
        ROS_INFO("[ImprovedRRT] ========== Planning Started ==========");
        bool success = rrt.solve(start_rrt, goal_rrt, path, timeout);
        ROS_INFO("[ImprovedRRT] ========== Planning %s ==========", success ? "SUCCEEDED" : "FAILED");
        
        if (success && !path.empty())
        {
            // 创建轨迹
            robot_trajectory::RobotTrajectoryPtr trajectory(
                new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));
            
            // 从当前场景状态开始，确保其他关节保持当前位置
            moveit::core::RobotState reference_state = getPlanningScene()->getCurrentState();
            
            // 直接使用RRT的原始路径点，不做手动插值
            // 让MoveIt的平滑算法来处理
            for (size_t i = 0; i < path.size(); ++i)
            {
                const auto& state = path[i];
                
                // 复制参考状态以保持其他关节的位置
                moveit::core::RobotState robot_state(reference_state);
                
                // 只更新规划组的关节
                robot_state.setJointGroupPositions(joint_model_group, state.values);
                robot_state.update();
                
                // 确保状态有效
                if (!robot_state.satisfiesBounds(joint_model_group))
                {
                    robot_state.enforceBounds(joint_model_group);
                }
                

                
                trajectory->addSuffixWayPoint(robot_state, 0.0);
            }
            
            std::size_t original_count = trajectory->getWayPointCount();
            
            // 步骤1：路径简化（Shortcutting）- 模仿OMPL的路径简化
            // 尝试用直线连接不相邻的路径点，跳过中间的弯路
            robot_trajectory::RobotTrajectoryPtr simplified_trajectory(
                new robot_trajectory::RobotTrajectory(robot_model_, getGroupName()));
            
            if (original_count > 2)
            {
                // 总是保留起点
                simplified_trajectory->addSuffixWayPoint(trajectory->getWayPoint(0), 0.0);
                
                std::size_t current_idx = 0;
                while (current_idx < original_count - 1)
                {
                    // 尝试从当前点跳到尽可能远的点
                    std::size_t farthest_valid = current_idx + 1;
                    
                    for (std::size_t test_idx = original_count - 1; test_idx > current_idx + 1; --test_idx)
                    {
                        // 检查从current_idx到test_idx的直线路径是否有效
                        const moveit::core::RobotState& state1 = trajectory->getWayPoint(current_idx);
                        const moveit::core::RobotState& state2 = trajectory->getWayPoint(test_idx);
                        
                        // 详细检查：在两点之间密集插值并检查碰撞
                        bool path_valid = true;
                        
                        // 首先检查终点是否碰撞
                        if (getPlanningScene()->isStateColliding(state2, getGroupName()))
                        {
                            path_valid = false;
                        }
                        else
                        {
                            // 计算需要的检查步数（基于距离）
                            std::vector<double> joint_values1, joint_values2;
                            state1.copyJointGroupPositions(joint_model_group, joint_values1);
                            state2.copyJointGroupPositions(joint_model_group, joint_values2);
                            
                            double distance = 0.0;
                            for (size_t j = 0; j < joint_values1.size(); ++j)
                            {
                                double diff = joint_values2[j] - joint_values1[j];
                                distance += diff * diff;
                            }
                            distance = std::sqrt(distance);
                            
                            // 每0.05弧度检查一次
                            const double resolution = 0.05;
                            int check_steps = std::max(5, static_cast<int>(std::ceil(distance / resolution)));
                            check_steps = std::min(check_steps, 50);  // 限制最大步数
                            
                            for (int step = 1; step < check_steps; ++step)
                            {
                                double t = static_cast<double>(step) / check_steps;
                                moveit::core::RobotState intermediate_state(robot_model_);
                                state1.interpolate(state2, t, intermediate_state);
                                
                                if (getPlanningScene()->isStateColliding(intermediate_state, getGroupName()))
                                {
                                    path_valid = false;
                                    break;
                                }
                            }
                        }
                        
                        if (path_valid)
                        {
                            farthest_valid = test_idx;
                            break;
                        }
                    }
                    
                    // 添加找到的最远有效点
                    if (farthest_valid != current_idx)
                    {
                        simplified_trajectory->addSuffixWayPoint(trajectory->getWayPoint(farthest_valid), 0.0);
                        current_idx = farthest_valid;
                    }
                    else
                    {
                        // 如果无法跳过，添加下一个点
                        current_idx++;
                        if (current_idx < original_count)
                        {
                            simplified_trajectory->addSuffixWayPoint(trajectory->getWayPoint(current_idx), 0.0);
                        }
                    }
                }
                
                trajectory = simplified_trajectory;
            }
            
            trajectory->unwind();  // 展开关节角度
            
            // 步骤2：使用样条插值进行平滑
            trajectory_processing::IterativeSplineParameterization spline_param;
            if (!spline_param.computeTimeStamps(*trajectory, 1.0, 1.0))
            {
                // 如果样条插值失败，使用抛物线插值作为后备
                trajectory_processing::IterativeParabolicTimeParameterization time_param;
                time_param.computeTimeStamps(*trajectory, 1.0, 1.0);
            }
            
            res.trajectory_ = trajectory;
            res.planning_time_ = (ros::Time::now() - start_time).toSec();
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
            
            ROS_INFO("[ImprovedRRT] Path: %zu waypoints -> %zu waypoints (%.3f sec)",
                    original_count, trajectory->getWayPointCount(), res.planning_time_);
            return true;
        }
        else
        {
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }
    }
    
    virtual bool terminate() override
    {
        return true;
    }
    
    virtual void clear() override
    {
    }
    
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
ImprovedRRTPlannerManager::ImprovedRRTPlannerManager()
    : nh_("~")
{
}

bool ImprovedRRTPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                          const std::string& ns)
{
    robot_model_ = model;
    return true;
}

void ImprovedRRTPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
    algs.clear();
    algs.push_back("ImprovedRRT");
}

planning_interface::PlanningContextPtr ImprovedRRTPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    ROS_INFO("[ImprovedRRTPlannerManager] getPlanningContext() ENTER for group: %s", req.group_name.c_str());
    
    if (req.group_name.empty())
    {
        ROS_ERROR("[ImprovedRRTPlannerManager] No group specified for planning");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return planning_interface::PlanningContextPtr();
    }
    
    ROS_INFO("[ImprovedRRTPlannerManager] Creating ImprovedRRTPlanningContext...");
    planning_interface::PlanningContextPtr context(
        new ImprovedRRTPlanningContext("ImprovedRRT", req.group_name, robot_model_));
    ROS_INFO("[ImprovedRRTPlanningContext] Context created");
    
    ROS_INFO("[ImprovedRRTPlannerManager] Setting planning scene and request...");
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    ROS_INFO("[ImprovedRRTPlannerManager] getPlanningContext() EXIT - SUCCESS");
    return context;
}

bool ImprovedRRTPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    return !req.group_name.empty();
}

} // namespace improved_rrt_planner

// 注册插件
PLUGINLIB_EXPORT_CLASS(improved_rrt_planner::ImprovedRRTPlannerManager,
                       planning_interface::PlannerManager)

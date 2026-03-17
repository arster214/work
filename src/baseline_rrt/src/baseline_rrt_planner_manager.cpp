#include "baseline_rrt/baseline_rrt_planner_manager.h"
#include "baseline_rrt/basic_rrt.h"
#include "baseline_rrt/basic_rrt_star.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <pluginlib/class_list_macros.hpp>

namespace baseline_rrt
{

// Planning Context 实现
class BaselineRRTPlanningContext : public planning_interface::PlanningContext
{
public:
    BaselineRRTPlanningContext(const std::string& name,
                              const std::string& group,
                              const moveit::core::RobotModelConstPtr& model)
        : planning_interface::PlanningContext(name, group)
        , robot_model_(model)
        , range_(0.5)
    {
        ros::NodeHandle nh("~");
        
        // 读取参数
        std::string param_ns = "/move_group/planner_configs/" + name + "/";
        nh.param(param_ns + "range", range_, 0.5);
        
        ROS_INFO("[BaselineRRT] Initialized %s planner for group '%s'", name.c_str(), group.c_str());
    }
    
    virtual ~BaselineRRTPlanningContext() {}
    
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
            res.description_.push_back(getName());
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
        
        // 获取起始状态
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
        
        // 创建规划器实例
        std::unique_ptr<BasicRRT> planner;
        
        if (getName() == "BasicRRTStar")
        {
            auto rrt_star = std::make_unique<BasicRRTStar>();
            rrt_star->setRewireRadius(1.5);  // 大幅增大 rewiring 半径
            planner = std::move(rrt_star);
        }
        else  // BasicRRT
        {
            planner = std::make_unique<BasicRRT>();
        }
        
        planner->setStateDimension(start_joint_values.size());
        planner->setJointLimits(lower_limits, upper_limits);
        planner->setRange(range_);
        
        // 设置状态验证器
        // Use init-capture to create a RobotState that lives with the lambda
        planner->setStateValidityChecker([this, joint_model_group, scratch_state = moveit::core::RobotState(robot_model_)](const State& state) mutable -> bool {
            try
            {
                if (!getPlanningScene())
                {
                    ROS_ERROR("[BaselineRRT] PlanningScene is null");
                    return false;
                }
                scratch_state.setJointGroupPositions(joint_model_group, state.values);
                scratch_state.update();
                return !getPlanningScene()->isStateColliding(scratch_state, getGroupName());
            }
            catch (const std::exception& e)
            {
                ROS_ERROR("[BaselineRRT] Exception in state validity checker: %s", e.what());
                return false;
            }
        });
        
        // 设置运动验证器
        // Use init-capture for scratch states
        planner->setMotionValidator([this, joint_model_group, 
                                   state1 = moveit::core::RobotState(robot_model_), 
                                   state2 = moveit::core::RobotState(robot_model_), 
                                   intermediate_state = moveit::core::RobotState(robot_model_)](const State& from, const State& to) mutable -> bool {
            try
            {
                if (!getPlanningScene())
                {
                    ROS_ERROR("[BaselineRRT] PlanningScene is null");
                    return false;
                }
                
                state1.setJointGroupPositions(joint_model_group, from.values);
                state2.setJointGroupPositions(joint_model_group, to.values);
                state1.update();
                state2.update();
                
                // 检查终点状态
                if (getPlanningScene()->isStateColliding(state2, getGroupName()))
                    return false;
                
                // 计算距离
                double distance = 0.0;
                for (size_t j = 0; j < from.values.size(); ++j)
                {
                    double diff = to.values[j] - from.values[j];
                    distance += diff * diff;
                }
                distance = std::sqrt(distance);
                
                // 自适应检查步数
                const double resolution = 0.05;
                int num_steps = std::max(2, static_cast<int>(std::ceil(distance / resolution)));
                num_steps = std::min(num_steps, 50);
                
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
                ROS_ERROR("[BaselineRRT] Exception in motion validator: %s", e.what());
                return false;
            }
        });
        
        // 执行规划
        State start_rrt(start_joint_values);
        State goal_rrt(goal_joint_values);
        std::vector<State> path;
        
        double timeout = request_.allowed_planning_time;
        if (timeout <= 0.0) timeout = 30.0;  // BasicRRT needs more time without goal bias
        
        ROS_INFO("[BaselineRRT] ========== Planning Started ==========");
        bool success = planner->solve(start_rrt, goal_rrt, path, timeout);
        ROS_INFO("[BaselineRRT] ========== Planning %s ==========", success ? "SUCCEEDED" : "FAILED");
        ROS_INFO("[BaselineRRT] Generated %zu nodes", planner->getNodeCount());
        
        if (success && !path.empty())
        {
            // 创建轨迹
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
                {
                    robot_state.enforceBounds(joint_model_group);
                }
                
                trajectory->addSuffixWayPoint(robot_state, 0.0);
            }
            
            trajectory->unwind();
            
            // 时间参数化
            trajectory_processing::IterativeParabolicTimeParameterization time_param;
            time_param.computeTimeStamps(*trajectory, 1.0, 1.0);
            
            res.trajectory_ = trajectory;
            res.planning_time_ = (ros::Time::now() - start_time).toSec();
            res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
            
            ROS_INFO("[BaselineRRT] Path: %zu waypoints (%.3f sec)",
                    trajectory->getWayPointCount(), res.planning_time_);
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
};

// PlannerManager 实现
BaselineRRTPlannerManager::BaselineRRTPlannerManager()
    : nh_("~")
{
}

bool BaselineRRTPlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                          const std::string& ns)
{
    robot_model_ = model;
    return true;
}

void BaselineRRTPlannerManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
    algs.clear();
    algs.push_back("BasicRRT");
    algs.push_back("BasicRRTStar");
}

planning_interface::PlanningContextPtr BaselineRRTPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
    if (req.group_name.empty())
    {
        ROS_ERROR("[BaselineRRTPlannerManager] No group specified");
        error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
        return planning_interface::PlanningContextPtr();
    }
    
    planning_interface::PlanningContextPtr context(
        new BaselineRRTPlanningContext(req.planner_id, req.group_name, robot_model_));
    
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return context;
}

bool BaselineRRTPlannerManager::canServiceRequest(
    const planning_interface::MotionPlanRequest& req) const
{
    return !req.group_name.empty();
}

} // namespace baseline_rrt

// 注册插件
PLUGINLIB_EXPORT_CLASS(baseline_rrt::BaselineRRTPlannerManager,
                       planning_interface::PlannerManager)

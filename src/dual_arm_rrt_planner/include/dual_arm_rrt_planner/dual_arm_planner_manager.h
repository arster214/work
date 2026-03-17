#ifndef DUAL_ARM_RRT_PLANNER_DUAL_ARM_PLANNER_MANAGER_H
#define DUAL_ARM_RRT_PLANNER_DUAL_ARM_PLANNER_MANAGER_H

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>

namespace dual_arm_planner
{

class DualArmPlannerManager : public planning_interface::PlannerManager
{
public:
    DualArmPlannerManager();
    
    virtual ~DualArmPlannerManager() {}
    
    virtual bool initialize(const moveit::core::RobotModelConstPtr& model,
                           const std::string& ns) override;
    
    virtual std::string getDescription() const override
    {
        return "Dual-Arm RRT Planner";
    }
    
    virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const override;
    
    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const override;
    
    virtual bool canServiceRequest(
        const planning_interface::MotionPlanRequest& req) const override;
    
private:
    moveit::core::RobotModelConstPtr robot_model_;
    ros::NodeHandle nh_;
};

} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_DUAL_ARM_PLANNER_MANAGER_H

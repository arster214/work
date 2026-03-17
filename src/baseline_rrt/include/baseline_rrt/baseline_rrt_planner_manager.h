#ifndef BASELINE_RRT_BASELINE_RRT_PLANNER_MANAGER_H
#define BASELINE_RRT_BASELINE_RRT_PLANNER_MANAGER_H

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

namespace baseline_rrt
{

/**
 * @brief MoveIt Planner Manager for Baseline RRT planners
 */
class BaselineRRTPlannerManager : public planning_interface::PlannerManager
{
public:
    BaselineRRTPlannerManager();
    
    virtual ~BaselineRRTPlannerManager() {}
    
    virtual bool initialize(const moveit::core::RobotModelConstPtr& model,
                           const std::string& ns) override;
    
    virtual bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override;
    
    virtual std::string getDescription() const override
    {
        return "Baseline RRT";
    }
    
    virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const override;
    
    virtual planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::MoveItErrorCodes& error_code) const override;
    
private:
    ros::NodeHandle nh_;
    moveit::core::RobotModelConstPtr robot_model_;
};

} // namespace baseline_rrt

#endif // BASELINE_RRT_BASELINE_RRT_PLANNER_MANAGER_H

#ifndef DUAL_ARM_RRT_PLANNER_MODULE_BASE_H
#define DUAL_ARM_RRT_PLANNER_MODULE_BASE_H

#include <memory>
#include <string>
#include <ros/ros.h>
#include "dual_arm_rrt_planner/planning_context.h"

namespace dual_arm_planner
{
namespace modules
{

class ModuleBase
{
public:
    virtual ~ModuleBase() = default;
    virtual bool initialize(const ros::NodeHandle& nh, const std::string& name) = 0;
    virtual bool preSample(PlanningContext& context) { return false; }
    virtual void postSample(PlanningContext& context) { }
    virtual void preExtend(PlanningContext& context) { }
    virtual void postExtend(PlanningContext& context) { }
    virtual void cleanup() { }
    virtual std::string getName() const = 0;

protected:
    ros::NodeHandle nh_;
    std::string name_;
};

using ModuleBasePtr = std::shared_ptr<ModuleBase>;

} // namespace modules
} // namespace dual_arm_planner

#endif // DUAL_ARM_RRT_PLANNER_MODULE_BASE_H

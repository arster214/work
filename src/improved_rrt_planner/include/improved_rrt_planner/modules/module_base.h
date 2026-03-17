#ifndef IMPROVED_RRT_PLANNER_MODULE_BASE_H
#define IMPROVED_RRT_PLANNER_MODULE_BASE_H

#include <memory>
#include <string>
#include <ros/ros.h>
#include "improved_rrt_planner/planning_context.h"

namespace improved_rrt_planner
{
namespace modules
{

/**
 * @brief 模块基类 - 定义所有功能模块必须实现的统一接口
 * 
 * 所有改进算法模块（目标偏置、人工势场、节点拒绝、可变步长等）
 * 都必须继承此基类并实现相应的接口方法。
 */
class ModuleBase
{
public:
    virtual ~ModuleBase() = default;
    
    /**
     * @brief 初始化模块
     * @param nh ROS节点句柄，用于读取参数
     * @param name 模块名称
     * @return 初始化成功返回true，失败返回false
     */
    virtual bool initialize(const ros::NodeHandle& nh, const std::string& name) = 0;
    
    /**
     * @brief 采样前处理 - 模块可以提供自定义采样点
     * @param context 规划上下文
     * @return 如果模块提供了采样点返回true，否则返回false
     * 
     * 例如：Goal_Bias_Module 可以在此方法中返回目标状态
     */
    virtual bool preSample(PlanningContext& context) 
    { 
        return false; 
    }
    
    /**
     * @brief 采样后处理 - 在采样后但扩展前调用
     * @param context 规划上下文
     */
    virtual void postSample(PlanningContext& context) 
    {
        // 默认不做任何处理
    }
    
    /**
     * @brief 扩展前处理 - 模块可以调整扩展参数
     * @param context 规划上下文
     * 
     * 例如：Variable_Step_Module 可以在此方法中调整 step_size
     */
    virtual void preExtend(PlanningContext& context) 
    {
        // 默认不做任何处理
    }
    
    /**
     * @brief 扩展后处理 - 模块可以验证/修改新节点
     * @param context 规划上下文
     * 
     * 例如：APF_Module 可以调整节点位置
     *       T_RRT_Module 可以拒绝高代价节点
     *       Node_Rejection_Module 可以拒绝冗余节点
     */
    virtual void postExtend(PlanningContext& context) 
    {
        // 默认不做任何处理
    }
    
    /**
     * @brief 清理 - 规划完成时调用
     */
    virtual void cleanup() 
    {
        // 默认不做任何处理
    }
    
    /**
     * @brief 获取模块名称（用于日志）
     * @return 模块名称
     */
    virtual std::string getName() const = 0;

protected:
    ros::NodeHandle nh_;      // ROS节点句柄
    std::string name_;        // 模块名称
};

// 模块智能指针类型定义
using ModuleBasePtr = std::shared_ptr<ModuleBase>;

} // namespace modules
} // namespace improved_rrt_planner

#endif // IMPROVED_RRT_PLANNER_MODULE_BASE_H

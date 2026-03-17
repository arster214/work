#!/usr/bin/env python3
# Quick test to force move_group to instantiate the DualArmRRT planner
# Run this after move_group is up: rosrun dual_arm_rrt_planner test_dual_rrt_planner.py

import sys
import rospy
from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander, RobotCommander

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('test_dual_rrt_planner', anonymous=True)

    rospy.sleep(1.0)
    robot = RobotCommander()
    groups = robot.get_group_names()
    print('Available groups:', groups)

    group_name = 'dual_arms'
    if group_name not in groups:
        print('ERROR: planning group "dual_arms" not found. Aborting.')
        roscpp_shutdown()
        sys.exit(1)

    # Print rosparams related to pipelines
    try:
        pipelines = rospy.get_param('/move_group/planning_pipelines')
    except Exception as e:
        pipelines = None
    print('/move_group/planning_pipelines =', pipelines)

    try:
        cfg = rospy.get_param('/move_group/planning_pipelines/dual_rrt/planner_configs')
    except Exception as e:
        cfg = None
    print('/move_group/planning_pipelines/dual_rrt/planner_configs =', cfg)

    group = MoveGroupCommander(group_name)
    print('End effector link:', group.get_end_effector_link())

    # Force planner id to DualArmRRT and attempt a trivial plan (current -> current)
    planner_id = 'DualArmRRT'
    print('Setting planner_id to', planner_id)
    group.set_planner_id(planner_id)
    group.set_start_state_to_current_state()

    current = group.get_current_joint_values()
    print('Current joint values:', current)

    # Set goal equal to current state for a trivial plan
    group.set_joint_value_target(current)

    rospy.loginfo('Requesting plan (this should trigger DualArmRRT instantiation in move_group)')
    plan = group.plan()
    print('Plan result (type):', type(plan))
    # Different MoveIt versions return different structures; try to detect success
    success = False
    try:
        if hasattr(plan, 'joint_trajectory') and plan.joint_trajectory.points:
            success = True
    except Exception:
        pass
    try:
        # Some MoveIt returns (bool, plan)
        if isinstance(plan, tuple) and len(plan) >= 1 and isinstance(plan[0], bool):
            success = plan[0]
    except Exception:
        pass

    print('Plan success (heuristic):', success)

    roscpp_shutdown()

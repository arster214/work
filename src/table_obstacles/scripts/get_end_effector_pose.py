#!/usr/bin/env python3
"""
Print current end-effector pose(s) for the robot using MoveIt (moveit_commander).

Usage:
  rosrun table_obstacles get_end_effector_pose.py            # auto-detect left/right groups and print
  rosrun table_obstacles get_end_effector_pose.py _group:=l_arm  # print specific group (via rosparam)

This script prints translation and quaternion (x,y,z,w) for each detected end effector.
"""
from __future__ import print_function
import rospy
import sys
import time
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander


def find_arm_groups(robot):
    groups = robot.get_group_names()
    left = None
    right = None
    for g in groups:
        gl = g.lower()
        if 'left' in gl or 'l_arm' in gl or gl.startswith('l_') or gl.startswith('l'):
            if left is None:
                left = g
        if 'right' in gl or 'r_arm' in gl or gl.startswith('r_') or gl.startswith('r'):
            if right is None:
                right = g
    return left, right


def print_pose(label, pose_stamped):
    p = pose_stamped.pose.position
    q = pose_stamped.pose.orientation
    print('--- %s ---' % label)
    print(' position: x={:.4f} y={:.4f} z={:.4f}'.format(p.x, p.y, p.z))
    print(' orientation (quat): x={:.6f} y={:.6f} z={:.6f} w={:.6f}'.format(q.x, q.y, q.z, q.w))


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_end_effector_pose', anonymous=True)

    # small delay to ensure move_group is up if this is run immediately after launch
    rospy.sleep(1.0)

    robot = RobotCommander()

    # allow user to override group via rosparam _group:=group_name or ROS remap
    group_override = rospy.get_param('~group', None)

    if group_override:
        try:
            group = MoveGroupCommander(group_override)
            eef = group.get_end_effector_link()
            pose = group.get_current_pose(eef)
            print_pose(group_override + ' (' + eef + ')', pose)
        except Exception as e:
            print('Failed to read group {}: {}'.format(group_override, e))
        return

    left, right = find_arm_groups(robot)
    printed = False
    if left:
        try:
            g = MoveGroupCommander(left)
            eef = g.get_end_effector_link()
            pose = g.get_current_pose(eef)
            print_pose(left + ' (' + eef + ')', pose)
            printed = True
        except Exception as e:
            print('Error reading left group {}: {}'.format(left, e))

    if right:
        try:
            g = MoveGroupCommander(right)
            eef = g.get_end_effector_link()
            pose = g.get_current_pose(eef)
            print_pose(right + ' (' + eef + ')', pose)
            printed = True
        except Exception as e:
            print('Error reading right group {}: {}'.format(right, e))

    if not printed:
        # fallback: print groups available
        print('No left/right groups auto-detected. Available groups:', robot.get_group_names())


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

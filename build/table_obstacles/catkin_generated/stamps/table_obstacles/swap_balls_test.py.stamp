#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Swap two balls on a table using two arms via MoveIt.

Usage: run this node after move_group is up (or include it in a launch with a short sleep).

Behaviour:
- Add two sphere obstacles (green, red) and a thin box barrier between them.
- Plan pick-and-place for left arm: pick left (green) -> place on right position.
- Plan pick-and-place for right arm: pick right (red) -> place on left position.

Notes:
- Robot is assumed to face -Y and left hand is +X (this script places objects accordingly).
- The script tries to auto-detect planning group names containing 'left'/'right' or fallbacks.
"""
from __future__ import print_function
import rospy
import sys
import copy
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander
import moveit_commander


def find_arm_groups(robot):
    groups = robot.get_group_names()
    left = None
    right = None
    for g in groups:
        gl = g.lower()
        if 'left' in gl or g.startswith('l_') or gl.startswith('l') or 'l_arm' in gl or 'larm' in gl:
            if left is None:
                left = g
        if 'right' in gl or g.startswith('r_') or gl.startswith('r') or 'r_arm' in gl or 'rarm' in gl:
            if right is None:
                right = g
    # fallback common names
    if left is None and 'l_arm' in groups:
        left = 'l_arm'
    if right is None and 'r_arm' in groups:
        right = 'r_arm'
    return left, right


def make_pose(x, y, z, q=(0, 0, 0, 1), frame='base_link_underpan'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    return p


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('swap_balls_test', anonymous=True)

    rospy.loginfo('Waiting a few seconds for MoveIt to be ready...')
    rospy.sleep(3.0)

    robot = RobotCommander()
    scene = PlanningSceneInterface(synchronous=True)

    left_group_name, right_group_name = find_arm_groups(robot)
    rospy.loginfo('Detected groups: left=%s right=%s', left_group_name, right_group_name)
    if not left_group_name or not right_group_name:
        rospy.logerr('Could not detect left/right planning groups. Available: %s', robot.get_group_names())
        return

    left_group = MoveGroupCommander(left_group_name)
    right_group = MoveGroupCommander(right_group_name)

    # End effector links
    left_eef = left_group.get_end_effector_link()
    right_eef = right_group.get_end_effector_link()
    rospy.loginfo('EEs: left=%s right=%s', left_eef, right_eef)

    # Define object positions (relative to base_link_underpan)
    # Robot faces -Y; left hand is +X
    table_z = 1.35  # typical table center Z used in existing scripts
    ball_radius = 0.04

    # 调整：将小球靠近中心以便机械臂可达，减小挡板尺寸
    left_ball_pos = (0.18, -0.78, table_z + ball_radius)
    right_ball_pos = (-0.18, -0.78, table_z + ball_radius)
    barrier_pos = (0.0, -0.78, table_z + 0.02)  # thin box sitting on table

    # Clear previous objects of same name
    for name in ['green_ball', 'red_ball', 'barrier_box']:
        try:
            scene.remove_world_object(name)
        except Exception:
            pass

    rospy.sleep(0.5)

    # Add spheres
    green_pose = make_pose(*left_ball_pos)
    red_pose = make_pose(*right_ball_pos)
    scene.add_sphere('green_ball', green_pose, radius=ball_radius)
    scene.add_sphere('red_ball', red_pose, radius=ball_radius)
    rospy.loginfo('Added green_ball and red_ball to planning scene')

    # Add barrier (thin tall box)
    barrier_pose = make_pose(*barrier_pos)
    # 缩短挡板长度与厚度，降低高度以便手臂可以伸过
    barrier_size = (0.03, 0.25, 0.10)  # x,y,z
    scene.add_box('barrier_box', barrier_pose, size=barrier_size)
    rospy.loginfo('Added barrier_box to planning scene')

    rospy.sleep(1.0)  # ensure scene updates

    def pick_and_place(group, eef_link, object_name, pick_pose, place_pose, attach_size=0.06):
        rospy.loginfo('Planning pick for %s', object_name)
        # approach above
        approach = copy.deepcopy(pick_pose)
        approach.pose.position.z += 0.10
        group.set_pose_target(approach, eef_link)
        plan_ok = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.3)

        # lower to grasp
        grasp = copy.deepcopy(pick_pose)
        grasp.pose.position.z += 0.02
        group.set_pose_target(grasp, eef_link)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.2)

        # attach (we use attach_box with small cube to simulate grasp)
        scene.attach_box(eef_link, object_name, touch_links=[eef_link], size=(attach_size, attach_size, attach_size))
        rospy.loginfo('Attached %s to %s', object_name, eef_link)
        rospy.sleep(0.5)

        # lift
        lift = copy.deepcopy(grasp)
        lift.pose.position.z += 0.12
        group.set_pose_target(lift, eef_link)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.3)

        # move to place approach
        place_approach = copy.deepcopy(place_pose)
        place_approach.pose.position.z += 0.12
        group.set_pose_target(place_approach, eef_link)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.3)

        # lower and detach
        place_down = copy.deepcopy(place_pose)
        place_down.pose.position.z += 0.02
        group.set_pose_target(place_down, eef_link)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.2)

        # detach and add world object at place
        scene.remove_attached_object(eef_link, name=object_name)
        rospy.loginfo('Detached %s from %s', object_name, eef_link)
        # add sphere at place
        scene.add_sphere(object_name, place_pose, radius=ball_radius)
        rospy.sleep(0.5)

    # Build PoseStamped for pick/place
    pick_left = make_pose(*left_ball_pos)
    place_right = make_pose(*right_ball_pos)
    pick_right = make_pose(*right_ball_pos)
    place_left = make_pose(*left_ball_pos)

    rospy.loginfo('Starting sequential swap: left->right then right->left')

    # Left arm: pick green -> place right
    pick_and_place(left_group, left_eef, 'green_ball', pick_left, place_right)

    # Right arm: pick red -> place left
    pick_and_place(right_group, right_eef, 'red_ball', pick_right, place_left)

    rospy.loginfo('Swap complete')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

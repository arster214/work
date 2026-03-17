#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import sys
import select
import termios
import tty
import datetime
import subprocess
from std_msgs.msg import Bool

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def save_octomap():
    rospy.loginfo("Manual scan finished, auto-saving Octomap...")

    home_dir = os.environ['HOME']
    workspace_dir = os.path.join(home_dir, "catkin_wsfrj")
    map_dir = os.path.join(workspace_dir, "map")

    if not os.path.exists(map_dir):
        os.makedirs(map_dir)
        rospy.loginfo(f"Created map directory: {map_dir}")

    today_str = datetime.datetime.now().strftime("%Y%m%d")
    max_id = 0
    files = [f for f in os.listdir(map_dir) if f.startswith(today_str) and f.endswith(".bt")]
    for f in files:
        try:
            base_name = os.path.splitext(f)[0]
            num_part = base_name[len(today_str):]
            if num_part.isdigit():
                max_id = max(max_id, int(num_part))
        except Exception:
            continue

    next_id = max_id + 1
    filename = f"{today_str}{next_id:02d}.bt"
    full_path = os.path.join(map_dir, filename)
    rospy.loginfo(f"Saving map to: {full_path}")

    cmd = [
        "rosrun", "octomap_server", "octomap_saver", full_path,
        "octomap_binary:=/octomap_binary"
    ]

    try:
        subprocess.check_call(cmd)
        rospy.loginfo(f"Successfully saved map to {filename}")
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to save map: {e}")

def main():
    rospy.init_node('manual_cloud_control', anonymous=True)
    
    # 针对左臂、右臂和头部的处理器节点单独发布使能话题
    pub_l = rospy.Publisher('/camera_l_processor/enable', Bool, queue_size=1, latch=True)
    pub_r = rospy.Publisher('/camera_r_processor/enable', Bool, queue_size=1, latch=True)
    pub_h = rospy.Publisher('/camera_head_processor/enable', Bool, queue_size=1, latch=True)
    
    # 默认状态：启动时关闭点云，等待用户手动开启
    state_l = False
    state_r = False
    state_h = False
    
    # 稍微延迟以确保发布者注册成功
    rospy.sleep(0.5)
    pub_l.publish(Bool(state_l))
    pub_r.publish(Bool(state_r))
    pub_h.publish(Bool(state_h))
    
    settings = termios.tcgetattr(sys.stdin)
    
    print("="*50)
    print("      Manual Point Cloud Control")
    print("--------------------------------------------------")
    print(" Press 'l' or 'L' : Toggle LEFT camera cloud (ON/OFF)")
    print(" Press 'r' or 'R' : Toggle RIGHT camera cloud (ON/OFF)")
    print(" Press 'h' or 'H' : Toggle HEAD camera cloud (ON/OFF)")
    print(" Press 'q' or 'Esc': Quit")
    print("="*50)
    print(f"Status -> L: {'[ON]' if state_l else '[OFF]'} | R: {'[ON]' if state_r else '[OFF]'} | H: {'[ON]' if state_h else '[OFF]'}\r", end="")
    
    try:
        while not rospy.is_shutdown():
            key = getKey(settings)
            
            update = False
            if key.lower() == 'l':
                state_l = not state_l
                pub_l.publish(Bool(state_l))
                update = True
            elif key.lower() == 'r':
                state_r = not state_r
                pub_r.publish(Bool(state_r))
                update = True
            elif key.lower() == 'h':
                state_h = not state_h
                pub_h.publish(Bool(state_h))
                update = True
            elif key.lower() == 'q' or key == '\x1b' or key == '\x03':
                break
                
            if update:
                print(f"\r\033[KStatus -> L: {'[ON]' if state_l else '[OFF]'} | R: {'[ON]' if state_r else '[OFF]'} | H: {'[ON]' if state_h else '[OFF]'}", end="")
                
            rospy.sleep(0.05)
            
    except Exception as e:
        print(e)
    finally:
        pub_l.publish(Bool(False))
        pub_r.publish(Bool(False))
        pub_h.publish(Bool(False))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        save_octomap()
        print("\nExited manual control.")

if __name__ == '__main__':
    main()

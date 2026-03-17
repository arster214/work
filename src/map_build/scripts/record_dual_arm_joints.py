#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys
import os
import select
import termios
import tty
import threading
import time
import json
import datetime
from sensor_msgs.msg import JointState

# Global storage for latest joint states
latest_left_state = None
latest_right_state = None
state_lock = threading.Lock()

def left_callback(msg):
    global latest_left_state
    with state_lock:
        latest_left_state = msg

def right_callback(msg):
    global latest_right_state
    with state_lock:
        latest_right_state = msg

def getKey():
    # Non-blocking key read
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def save_current_pose(filepath):
    with state_lock:
        l_state = latest_left_state
        r_state = latest_right_state

    if l_state is None or r_state is None:
        print("\r\033[K[WARN] Waiting for joint states... (Moving arm?)")
        return

    # Prepare data dictionary
    # Using simple list for positions for readability
    entry = {
        "timestamp": rospy.Time.now().to_sec(),
        "left": {
            "names": l_state.name,
            "position": list(l_state.position)
        },
        "right": {
            "names": r_state.name,
            "position": list(r_state.position)
        }
    }

    try:
        with open(filepath, 'a') as f:
            f.write(json.dumps(entry) + "\n")
        print(f"\r\033[K[INFO] Recorded pose to {filepath}")
    except Exception as e:
        print(f"\r\033[K[ERROR] Failed to write file: {e}")

if __name__ == "__main__":
    rospy.init_node('dual_arm_recorder')
    
    # Subscribe to the topics (using standard topics as seen in other files)
    rospy.Subscriber('/l_arm/joint_states', JointState, left_callback)
    rospy.Subscriber('/r_arm/joint_states', JointState, right_callback)

    # Output file configuration
    pose_dir = os.path.join(os.getcwd(), "pose")
    if not os.path.exists(pose_dir):
        os.makedirs(pose_dir)
        print(f"Created directory: {pose_dir}")

    # Generate filename: YYYYMMDDxx.jsonl
    today_str = datetime.datetime.now().strftime("%Y%m%d")
    
    # Find the next available index
    max_id = 0
    if os.path.exists(pose_dir):
        for f in os.listdir(pose_dir):
            if f.startswith(today_str) and f.endswith(".jsonl"):
                try:
                    # Filename structure: YYYYMMDDxx.jsonl
                    # Remove extension and date prefix to get the number
                    base_name = os.path.splitext(f)[0]
                    num_part = base_name[len(today_str):]
                    if num_part.isdigit():
                        max_id = max(max_id, int(num_part))
                except Exception:
                    continue
    
    next_id = max_id + 1
    filename = f"{today_str}{next_id:02d}.jsonl"
    output_file = os.path.join(pose_dir, filename)
    
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    print("="*50)
    print(f"Dual Arm Joint Recorder")
    print(f"Subscribed to: /l_arm/joint_states, /r_arm/joint_states")
    print(f"Output file:   {os.path.abspath(output_file)}")
    print("-" * 50)
    print("Press 's' to Record current pose")
    print("Press 'q' to Quit")
    print("="*50)

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == 's':
                save_current_pose(output_file)
            elif key == 'q' or key == '\x03': # q or ctrl-c
                break
            
            # rate limiting for loop
            time.sleep(0.05)
            
    except Exception as e:
        print(e)
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nRecorder stopped.")

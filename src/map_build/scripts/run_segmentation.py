#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import glob
import rospy
import subprocess

def get_latest_bt_file():
    map_dir = os.path.join(os.environ['HOME'], 'catkin_wsfrj/map')
    files = glob.glob(os.path.join(map_dir, '*.bt'))
    if not files:
        return None
    files.sort(key=os.path.getmtime)
    return files[-1]

if __name__ == '__main__':
    latest_bt = get_latest_bt_file()
    if not latest_bt:
        print("未找到任何 .bt 文件在 ~/catkin_wsfrj/map/ 中！")
        exit(1)
        
    print(f"找到最新地图: {latest_bt}")
    cmd = [
        "roslaunch", 
        "map_build", 
        "tabletop_segmentation.launch", 
        f"bt_file_path:={latest_bt}"
    ]
    subprocess.call(cmd)

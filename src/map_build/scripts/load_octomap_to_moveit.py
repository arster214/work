#!/usr/bin/env python3
import rospy
import os
import glob
import subprocess
import time
from octomap_msgs.msg import Octomap
from moveit_msgs.msg import PlanningScene
from geometry_msgs.msg import Pose

class OctomapLoader:
    def __init__(self):
        rospy.init_node('load_octomap_to_moveit')
        
        self.pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1)
        
        # 1. Find the latest map file
        cwd = os.getcwd()
        map_dir = os.path.join(cwd, "map")
        latest_map_file = self.get_latest_map(map_dir)
        
        if latest_map_file:
            rospy.loginfo(f"Found latest map: {latest_map_file}")
            # 2. Launch octomap_server to publish this map
            # We use a unique topic internal to this node logic to avoid conflicts if needed, 
            # but since we want to load it into MoveIt, listening to /octomap_binary is standard.
            # However, if build_master is running, there might be a conflict. 
            # Assuming this script is run INSTEAD of build_master or in a separate session.
            self.launch_map_server(latest_map_file)
        else:
            rospy.logwarn("No map file found in 'map/' folder. Waiting for /octomap_binary from elsewhere...")

        self.sub = rospy.Subscriber('/octomap_binary', Octomap, self.octomap_callback)
        
        self.latest_map = None
        self.timer = rospy.Timer(rospy.Duration(2.0), self.timer_callback)
        
    def get_latest_map(self, map_dir):
        if not os.path.exists(map_dir):
            return None
        list_of_files = glob.glob(os.path.join(map_dir, '*.bt')) 
        if not list_of_files:
            return None
        return max(list_of_files, key=os.path.getctime)

    def launch_map_server(self, map_file):
        # Run: rosrun octomap_server octomap_server_node <map_file>
        # We need to set the frame_id. Usually 'base_link_underpan' or 'world'.
        # Assuming 'base_link_underpan' as per build_master.launch
        cmd = [
            "rosrun", "octomap_server", "octomap_server_node", 
            map_file,
            "_frame_id:=base_link_underpan",
            "_latch:=true" # Make sure it sticks
        ]
        rospy.loginfo(f"Launching octomap_server for {map_file}...")
        self.server_process = subprocess.Popen(cmd)
        
        # Register cleanup
        rospy.on_shutdown(self.kill_server)

    def kill_server(self):
        if hasattr(self, 'server_process'):
            rospy.loginfo("Killing octomap_server process...")
            self.server_process.terminate()

    def octomap_callback(self, msg):
        rospy.loginfo_once("Received Octomap binary message. Will publish to PlanningScene periodically.")
        self.latest_map = msg

    def timer_callback(self, event):
        if self.latest_map is None:
            return
            
        # Create PlanningScene message
        scene = PlanningScene()
        scene.is_diff = True
        
        # Set the octomap
        scene.world.octomap.header = self.latest_map.header
        scene.world.octomap.octomap = self.latest_map
        
        # Set the origin of the octomap to identity
        scene.world.octomap.origin.position.x = 0
        scene.world.octomap.origin.position.y = 0
        scene.world.octomap.origin.position.z = 0
        scene.world.octomap.origin.orientation.w = 1.0
        
        # Publish to /planning_scene
        self.pub.publish(scene)


if __name__ == '__main__':
    try:
        loader = OctomapLoader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

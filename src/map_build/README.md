# real_robot_deployer

Small package to publish real-world obstacles into MoveIt Planning Scene. This contains a minimal example node that publishes a sample CollisionObject and subscribes to `/octomap_full` for future integration.

Usage
-----

1. Build the workspace:

```bash
cd /home/fang/catkin_ws
catkin_make
source devel/setup.bash
```

2. Launch the node:

```bash
roslaunch real_robot_deployer real_deploy.launch
```

3. In RViz with MoveIt, enable Planning Scene display to see the published collision object.

Next steps
----------
- Convert incoming octomap (`/octomap_full`) into `moveit_msgs::PlanningScene` world.octomap or into `moveit_msgs::CollisionObject` primitives.
- Use TF to transform sensor frames into the MoveIt planning frame before insertion.
- Add parameterization and safety checks for real robot execution.

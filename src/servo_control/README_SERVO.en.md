## **1. Project Introduction**

The servo ROS package changes the servo angle and retrieves the servo status through topics.

## **2. Code Structure**

servo\_control
├── README.md
├── servo\_demo
│   ├── CMakeLists.txt
│   ├── include
│   │   └── servo\_demo
│   ├── package.xml
│   ├── scripts
│   │   └── servo\_control\_demo.py A demo to test the servo ROS package, which changes the servo angle and retrieves the servo status by publishing and subscribing to topics.
│   └── src
└── servo\_ros
├── CMakeLists.txt
├── launch
│   └── servo\_start.launch Launch file to start the servo.
├── msg Custom servo messages
│   ├── Servo\_angle.msg
│   ├── Servo\_GetAngle.msg
│   └── Servo\_Move.msg
├── package.xml
└── src
└── servo\_controller.cpp Main logic code of the servo ROS package.

## **3. Compilation Method**

* Create Folder
  ```
  mkdir -p ~/catkin_ws/src
  ```
* Place the servo\_control folder into the workspace catkin\_ws/src/
* Compile the ROS package

  ```
  cd ~/catkin_ws
  catkin build
  ```

## **4. Run Instructions**

1\. Start the launch of the servo ROS package

```
cd ~/catkin_ws
source devel/setup.bash
rosluanch servo_ros servo_start.launch
```

2\. Start the demo of the package's functionality:

```
rosrun servo_demo servo_control_demo.py
```

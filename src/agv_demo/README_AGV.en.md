## **1. Project Introduction**

This package demonstrates how to call the Wushi chassis ROS interface. For more functionalities of the Wushi chassis, please refer to 'ROS Interface for Mobile Robots (External) - WOOSH Robotics.pdf'.

## **2. Code Structure**

```
├── agv_demo
│   ├── 底盘说明文件
│   │   ├── 底盘链接.txt
│   │   ├── 移动机器人的ROS接口（对外） - WOOSH Robotics.pdf   （悟时底盘接口文档）
│   │   ├── Woosh Design用户指南.pdf
│   │   ├── Woosh Design User Guide_ZH.pdf
│   │   ├── Woosh Mobile交互软件用户指南.pdf 
│   │   ├── Woosh Mobile User Guide_ZH.pdf
│   │   └── woosh_robot_sdk_interface_v1.1.62.pdf
│   ├── CMakeLists.txt
│   ├── include
│   │   └── agv_demo
│   ├── package.xml
│   ├── README_AGV.md
│   └── scripts
│       └── agv_demo.py     （调用悟时底盘ros接口样例代码）
```

## **3. Compilation Method**

* Create a folder

  ```
  mkdir -p ~/catkin_ws/src
  ```

* Place the agv\_demo folder into the workspace catkin\_ws/src/

* Compile the ROS package

  ```
  cd ~/catkin_ws
  catkin build
  source devel/setup.bash
  ```

## **4. Running Instructions**

Steps to run the chassis example

1\. Please refer to 'ROS Interface for Mobile Robots (External) - WOOSH Robotics.pdf' for environment variable configuration before use.

2\. Map and set the point 'A'

3\. Run the command

```
rosrun agv_demo agv_demo.py
```


## **一.项目介绍**
这个包时示范如何调用悟时底盘ros接口，更多悟时底盘功能请看《移动机器人的ROS接口（对外） - WOOSH Robotics.pdf》

## **二.代码结构**
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




## **三.编译方法**

- 创建 文件夹
    
    ```
    mkdir -p ~/catkin_ws/src
    ```
- 将 agv_demo文件夹放入工作空间catkin_ws/src/中
- 编译ros包

    ```
    cd ~/catkin_ws
    catkin build
    source devel/setup.bash
    ```
    
    

## **四.运行指令**


运行底盘样例步骤

1.使用前请参考《移动机器人的ROS接口（对外） - WOOSH Robotics.pdf》进行环境变量配置

2.建图并设置点位"A"

3.运行指令

```
rosrun agv_demo agv_demo.py
```


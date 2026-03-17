## **1. Project Introduction**

This functionality package encapsulates USB cameras and D435C cameras on embodied robots. Through topics, it can obtain the color frames from the USB camera and the color and depth frames from the D435C camera.

## **2. Code Structure**

```
├── d435_demo
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── d435_pub.launch  启动d435相机的launch
│   │   └── d435_sub.launch  订阅d435相机的launch
│   ├── package.xml
│   └── scripts
│       ├── __init__.py
│       ├── video_stream_pub.py  打开d435相机并发布颜色帧和深度帧话题
│       └── video_stream_sub.py  订阅d435相机颜色帧和深度帧话题
├── README_camera.md
└── usb_camera_demo
    ├── CMakeLists.txt
    ├── config
    │   └── camera_params.yaml
    ├── include
    │   └── usb_camera
    ├── launch
    │   ├── usb_camera_pub.launch 启动usb相机的launch
    │   └── usb_camera_sub.launch 订阅usb相机的launch
    ├── package.xml
    ├── scripts
    │   ├── video_stream_pub.py 打开usb相机并发布颜色帧话题
    │   └── video_stream_sub.py 订阅usb相机颜色帧话题
    └── src

```

## **3. Running Instructions**

### USB Camera

1\. Start the USB camera`ros`Launch of the functionality package

Open a new **terminal** You need to enter the workspace and source it before starting the program.

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch usb_camera_demo usb_camera_pub.launch idx:=0
```

​	Currently, the robot has two USB cameras. The idx parameter is used to specify which camera to open: 0 opens the first camera, and 1 opens the second camera. If the idx parameter is not specified, both USB cameras will be opened by default.

2\. Start demo example:

Display the color frame from the USB camera

```
source devel/setup.bash
roslaunch usb_camera_demo usb_camera_sub.launch 
```

### D435 Camera

1. Start the D435 camera`ros`Launch of the functionality package

Open a new **terminal** You need to enter the workspace and source it before starting the program.

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch d435_demo d435_pub.launch idx:=0
```

Currently, the robot has three D435 cameras. The idx parameter is used to specify which camera to open: 0 opens the first camera, 1 opens the second camera, and 2 opens the third camera. If the idx parameter is not specified, all three D435 cameras will be opened by default.

2\. Start demo example:

Display the color frame and depth frame of the D435 camera

```
roslaunch d435_demo d435_sub.launch 
```

## **一.项目介绍**

本功能包是对具身机器人上USB相机和D435C相机的封装，通过话题可以得到USB相机颜色帧和D435C相机的颜色帧和深度帧

## **二.代码结构**

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



## **三.运行指令**

### USB相机

1.启动usb相机`ros`功能包的launch

打开新的**终端**需要进入到工作空间下source后启动程序

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch usb_camera_demo usb_camera_pub.launch idx:=0
```



​	当前机器人有两个usb相机，idx参数用于指定打开哪个相机 0打开第一个相机 1打开第二个相机 如果不指定idx参数，默认打开两个usb相机



2.启动demo示例：

显示usb相机的颜色帧

```
source devel/setup.bash
roslaunch usb_camera_demo usb_camera_sub.launch 
```



### D435相机

1. 启动d435相机`ros`功能包的launch

打开新的**终端**需要进入到工作空间下source后启动程序

```
cd ~/catkin_ws
source devel/setup.bash
roslaunch d435_demo d435_pub.launch idx:=0
```



当前机器人有三个d435相机，idx参数用于指定打开哪个相机 0打开第一个相机 1打开第二个相机  2打开第三个相机 如果不指定idx参数，默认打开三个D435相机

2.启动demo示例：

显示d435相机的颜色帧和深度帧

```
roslaunch d435_demo d435_sub.launch 
```


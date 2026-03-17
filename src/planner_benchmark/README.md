# Planner Benchmark Package

用于路径规划算法性能测试和数据采集的 ROS 包。

## 功能

- 系统化测试多种路径规划算法
- 采集性能指标用于论文数据分析
- 支持 ImprovedRRT 和 OMPL 规划器对比
- 自动生成 CSV 格式的实验数据

## 包含的脚本

### 1. paper_data_collection.py
主要的数据采集脚本，用于：
- 测试多个目标姿态（每个重复 20 次）
- 对比 ImprovedRRT、OMPL RRT、OMPL RRTstar
- 采集规划时间、路径长度、成功率等指标
- 导出 CSV 格式的原始数据

### 2. check_available_metrics.py
检查 MoveIt/OMPL 能提供哪些性能指标的工具脚本

## 使用方法

### 1. 编译包

```bash
cd ~/dual_arm_robot_ws
catkin build planner_benchmark
source devel/setup.bash
```

### 2. 启动机器人和 MoveIt

```bash
# 终端 1: 启动 Gazebo 和机器人
roslaunch your_robot_description gazebo.launch

# 终端 2: 启动 MoveIt
roslaunch your_moveit_config move_group.launch
```

### 3. 运行数据采集

```bash
# 使用默认配置
rosrun planner_benchmark paper_data_collection.py

# 或指定参数
rosrun planner_benchmark paper_data_collection.py --runs 20 --timeout 10.0 --output ~/benchmark_results
```

## 输出文件

数据将保存到 `~/benchmark_results/` 目录：

- `benchmark_raw_YYYYMMDD_HHMMSS.csv` - 原始测试数据
- `benchmark_summary_YYYYMMDD_HHMMSS.csv` - 统计摘要

## 采集的性能指标

1. **规划时间** (planning_time) - 秒
2. **成功/失败** (success) - 布尔值
3. **路径长度** (path_length) - 关节空间欧氏距离
4. **路径点数量** (num_waypoints) - 整数
5. **平滑度** (smoothness) - 加速度变化量

## 测试配置

- **规划组**: r_arm (7-DOF)
- **起始姿态**: 全零 [0, 0, 0, 0, 0, 0, 0]
- **目标姿态**: 4 个预定义姿态
- **重复次数**: 每个姿态 20 次
- **规划器**: ImprovedRRT (T-RRT+RRT*), OMPL RRT, OMPL RRTstar

## 依赖

- ROS (Noetic)
- MoveIt
- Python 3
- numpy (可选，用于统计分析)
- matplotlib (可选，用于可视化)

## 注意事项

1. 确保机器人在 Gazebo 中正常运行
2. 确保 MoveIt move_group 节点已启动
3. 测试过程中不要手动移动机器人
4. 完整测试大约需要 1-2 小时

## 故障排除

### 问题: 规划总是失败
- 检查 MoveIt 是否正常运行
- 检查目标姿态是否在关节限制内
- 检查是否有碰撞

### 问题: 脚本无法连接到 MoveIt
- 确保 move_group 节点正在运行
- 检查规划组名称是否正确 (r_arm)

### 问题: 输出文件为空
- 检查输出目录权限
- 查看终端错误信息

## 许可证

BSD

# 使用指南

## 快速开始

### 1. 编译包

```bash
cd ~/dual_arm_robot_ws
catkin build planner_benchmark
source devel/setup.bash
```

### 2. 启动系统

**终端 1 - 启动 Gazebo 和机器人:**
```bash
roslaunch dual_arm_robot_gazebo dual_arm_robot_world.launch
```

**终端 2 - 启动 MoveIt:**
```bash
roslaunch dual_arm_robot_moveit_config move_group.launch
```

等待 MoveIt 完全启动（看到 "You can start planning now!" 消息）

### 3. 运行数据采集

**终端 3 - 运行数据采集脚本:**

```bash
# 方法 1: 使用 rosrun（推荐）
rosrun planner_benchmark paper_data_collection.py

# 方法 2: 使用 launch 文件
roslaunch planner_benchmark data_collection.launch

# 方法 3: 自定义参数
rosrun planner_benchmark paper_data_collection.py --runs 20 --timeout 10.0 --output ~/my_results
```

## 测试配置

### 默认配置

- **规划组**: r_arm (7-DOF)
- **起始姿态**: [0, 0, 0, 0, 0, 0, 0]
- **目标姿态**: 4 个
  - 姿态 1: [1.375, 0.318, 0.339, 1.215, -1.107, 0.905, -0.002]
  - 姿态 2: [1.862, 0.664, 0.345, 1.449, -1.457, 1.128, 0.849]
  - 姿态 3: [-1.012, -1.372, 2.331, 0.195, -2.949, -0.966, -3.475]
  - 姿态 4: [2.630, 0.850, -0.761, 1.432, -2.464, -0.535, 1.681]
- **规划器**: 3 个
  - ImprovedRRT (T-RRT+RRT*)
  - OMPL RRT
  - OMPL RRTstar
- **重复次数**: 每个姿态 20 次
- **超时时间**: 10 秒

### 修改配置

编辑配置文件：
```bash
rosed planner_benchmark test_poses.yaml
```

或创建自己的配置文件并使用：
```bash
rosrun planner_benchmark paper_data_collection.py --config /path/to/your/config.yaml
```

## 命令行参数

```bash
rosrun planner_benchmark paper_data_collection.py [选项]

选项:
  --config PATH     配置文件路径
  --runs N          每个姿态重复次数（默认: 20）
  --timeout SEC     规划超时时间（默认: 10.0）
  --output DIR      输出目录（默认: ~/benchmark_results）
  -h, --help        显示帮助信息
```

## 输出文件

结果保存在 `~/benchmark_results/` 目录：

### benchmark_raw_YYYYMMDD_HHMMSS.csv

原始测试数据，包含以下列：

| 列名 | 说明 | 单位 |
|------|------|------|
| timestamp | 测试时间戳 | - |
| planner_name | 规划器名称 | - |
| planner_display_name | 规划器显示名称 | - |
| pose_id | 姿态 ID | - |
| pose_description | 姿态描述 | - |
| run_number | 运行次数 | - |
| success | 是否成功 | True/False |
| planning_time | 规划时间 | 秒 |
| num_waypoints | 路径点数量 | 个 |
| path_length | 路径长度 | 弧度 |
| smoothness | 平滑度 | - |
| execution_time | 执行时间 | 秒 |

### 示例数据

```csv
timestamp,planner_name,planner_display_name,pose_id,pose_description,run_number,success,planning_time,num_waypoints,path_length,smoothness,execution_time
2024-01-01 12:00:00,ImprovedRRT,T-RRT+RRT* (Ours),pose_1,目标姿态 1,1,True,0.523,45,8.234,0.123,3.456
2024-01-01 12:00:15,ImprovedRRT,T-RRT+RRT* (Ours),pose_1,目标姿态 1,2,True,0.487,42,7.891,0.115,3.234
...
```

## 数据分析

使用 Python/Pandas 分析数据：

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
df = pd.read_csv('~/benchmark_results/benchmark_raw_20240101_120000.csv')

# 过滤成功的测试
df_success = df[df['success'] == True]

# 按规划器分组统计
stats = df_success.groupby('planner_display_name').agg({
    'planning_time': ['mean', 'std', 'min', 'max'],
    'path_length': ['mean', 'std'],
    'num_waypoints': ['mean', 'std']
})

print(stats)

# 绘制规划时间对比
df_success.boxplot(column='planning_time', by='planner_display_name')
plt.ylabel('Planning Time (s)')
plt.title('Planning Time Comparison')
plt.suptitle('')  # 移除默认标题
plt.show()
```

## 预计时间

- 单次测试: ~10-15 秒
- 单个姿态 20 次: ~3-5 分钟
- 4 个姿态: ~12-20 分钟
- 3 个规划器: **约 40-60 分钟**

## 故障排除

### 问题: 脚本无法启动

**解决方案:**
```bash
# 检查脚本权限
chmod +x ~/dual_arm_robot_ws/src/planner_benchmark/scripts/paper_data_collection.py

# 重新 source
source ~/dual_arm_robot_ws/devel/setup.bash
```

### 问题: 找不到规划组 'r_arm'

**解决方案:**
- 确保 MoveIt 已启动
- 检查规划组名称是否正确
- 修改配置文件中的 `group_name`

### 问题: 规划总是失败

**可能原因:**
1. 目标姿态超出关节限制
2. 目标姿态与障碍物碰撞
3. 规划超时时间太短

**解决方案:**
```bash
# 增加超时时间
rosrun planner_benchmark paper_data_collection.py --timeout 20.0

# 检查目标姿态是否可达
rosrun planner_benchmark check_available_metrics.py
```

### 问题: 机器人移动异常

**解决方案:**
- 重启 Gazebo 和 MoveIt
- 确保没有其他程序控制机器人
- 检查 Gazebo 物理引擎是否正常

### 问题: 数据文件为空

**解决方案:**
```bash
# 检查输出目录权限
ls -la ~/benchmark_results/

# 手动创建目录
mkdir -p ~/benchmark_results

# 指定其他输出目录
rosrun planner_benchmark paper_data_collection.py --output /tmp/benchmark_results
```

## 中断和恢复

如果测试被中断：

1. 已完成的测试会自动保存
2. 重新运行脚本会创建新的输出文件
3. 可以手动合并多个 CSV 文件

```python
import pandas as pd

# 合并多个 CSV 文件
df1 = pd.read_csv('benchmark_raw_20240101_120000.csv')
df2 = pd.read_csv('benchmark_raw_20240101_130000.csv')
df_combined = pd.concat([df1, df2], ignore_index=True)
df_combined.to_csv('benchmark_combined.csv', index=False)
```

## 高级用法

### 只测试特定规划器

修改配置文件，注释掉不需要的规划器：

```yaml
planners:
  - name: "ImprovedRRT"
    display_name: "T-RRT+RRT* (Ours)"
    # ...
  
  # - name: "RRT"  # 注释掉不测试
  #   display_name: "RRT (OMPL)"
```

### 添加更多目标姿态

在配置文件中添加：

```yaml
goal_poses:
  # ... 现有姿态
  
  - id: "pose_5"
    description: "目标姿态 5"
    joints: [1.0, 0.5, 0.3, 1.2, -1.1, 0.9, 0.0]
```

### 测试不同的规划器参数

```yaml
planners:
  - name: "ImprovedRRT"
    display_name: "T-RRT+RRT* (range=0.3)"
    params:
      enable_trrt: true
      enable_rrt_star: true
      range: 0.3  # 修改参数
      goal_bias: 0.05
```

## 技巧

1. **在 RViz 中监控**: 启动 RViz 可以实时看到机器人运动
2. **使用 screen/tmux**: 长时间测试建议使用 screen 或 tmux
3. **定期备份数据**: 测试过程中定期复制输出文件
4. **记录测试条件**: 记录测试时的系统状态、障碍物配置等

## 联系

如有问题，请查看：
- README.md
- 包文档
- ROS Wiki

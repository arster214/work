# Baseline RRT 测试指南

## 📋 测试步骤

### 1. 编译包

```bash
cd ~/dual_arm_robot_ws
catkin build baseline_rrt
source devel/setup.bash
```

### 2. 验证插件注册

```bash
rospack plugins --attrib=plugin moveit_core | grep baseline
```

**预期输出：**
```
baseline_rrt /path/to/baseline_rrt_planner_plugin.xml
```

如果没有输出，说明插件没有正确注册。

### 3. 启动系统

**终端 1 - 启动 Gazebo:**
```bash
roslaunch dual_arm_robot_gazebo dual_arm_robot_world.launch
```

**终端 2 - 启动 MoveIt:**
```bash
roslaunch dual_65B_arm_moveit_config move_group.launch
```

等待看到：
```
You can start planning now!
```

### 4. 测试规划器

**终端 3 - 运行测试脚本:**
```bash
chmod +x ~/dual_arm_robot_ws/src/baseline_rrt/scripts/test_baseline_rrt.py
rosrun baseline_rrt test_baseline_rrt.py
```

**预期输出：**
```
================================================================================
测试 Baseline RRT 规划器
================================================================================

当前规划组: r_arm
可用的规划器:
  ✓ BasicRRT - 可用
  ✓ BasicRRTStar - 可用

测试 BasicRRT 规划...
  开始规划...
  ✓ 规划成功!

================================================================================
测试完成!
================================================================================
```

### 5. 在 RViz 中测试

如果你有 RViz 运行：

1. 在 RViz 的 MotionPlanning 面板中
2. 找到 "Planning" 标签
3. 在 "Planner" 下拉菜单中
4. 应该能看到 **BasicRRT** 和 **BasicRRTStar**

选择其中一个，然后点击 "Plan" 按钮测试。

## 🔍 故障排除

### 问题 1: 找不到规划器

**症状：**
```
✗ BasicRRT - 不可用: ...
```

**解决方案：**

1. 检查编译是否成功：
```bash
catkin build baseline_rrt --verbose
```

2. 检查插件文件是否存在：
```bash
ls ~/dual_arm_robot_ws/src/baseline_rrt/baseline_rrt_planner_plugin.xml
```

3. 检查库文件是否生成：
```bash
ls ~/dual_arm_robot_ws/devel/lib/libbaseline_rrt_planner.so
```

4. 重新 source：
```bash
source ~/dual_arm_robot_ws/devel/setup.bash
```

5. 重启 MoveIt：
```bash
# 在终端 2 中按 Ctrl+C 停止 MoveIt
# 然后重新启动
roslaunch dual_65B_arm_moveit_config move_group.launch
```

### 问题 2: 编译错误

**常见错误：**

**错误 1: 找不到 OMPL**
```
Could not find OMPL
```

**解决：**
```bash
sudo apt-get install ros-noetic-ompl
```

**错误 2: 找不到 MoveIt 头文件**
```
fatal error: moveit/planning_interface/planning_interface.h: No such file or directory
```

**解决：**
```bash
sudo apt-get install ros-noetic-moveit-core ros-noetic-moveit-ros-planning-interface
```

### 问题 3: 规划总是失败

**可能原因：**
1. 超时时间太短（BasicRRT 没有目标偏置，需要更长时间）
2. 目标姿态不可达
3. 碰撞检测问题

**解决：**
```python
# 增加规划时间
group.set_planning_time(30.0)  # 30 秒

# 或者使用 BasicRRTStar（通常更快）
group.set_planner_id("BasicRRTStar")
```

### 问题 4: MoveIt 日志中看到错误

查看详细日志：
```bash
# 在启动 MoveIt 时添加日志级别
roslaunch dual_65B_arm_moveit_config move_group.launch --screen
```

查找包含 "BasicRRT" 或 "baseline_rrt" 的消息。

## ✅ 验证清单

在继续数据采集之前，确保：

- [ ] `catkin build baseline_rrt` 成功
- [ ] `rospack plugins` 能找到 baseline_rrt
- [ ] MoveIt 启动时没有关于 baseline_rrt 的错误
- [ ] 测试脚本显示 BasicRRT 和 BasicRRTStar 可用
- [ ] 至少一次规划成功

## 📊 性能预期

### BasicRRT
- **规划时间**: 5-30 秒（取决于场景复杂度）
- **成功率**: 60-80%（无目标偏置，较低）
- **路径质量**: 一般

### BasicRRTStar
- **规划时间**: 10-40 秒（rewiring 需要额外时间）
- **成功率**: 70-85%
- **路径质量**: 较好（有 rewiring 优化）

### ImprovedRRT (T-RRT+RRT*)
- **规划时间**: 2-10 秒（有目标偏置和优化）
- **成功率**: 90-95%
- **路径质量**: 最好

## 🎯 下一步

测试成功后，可以：

1. 运行完整的数据采集：
```bash
rosrun planner_benchmark paper_data_collection.py
```

2. 或者手动测试不同的姿态：
```python
import moveit_commander

group = moveit_commander.MoveGroupCommander("r_arm")

# 测试 BasicRRT
group.set_planner_id("BasicRRT")
group.set_joint_value_target([1.375, 0.318, 0.339, 1.215, -1.107, 0.905, -0.002])
plan = group.plan()

# 测试 BasicRRTStar
group.set_planner_id("BasicRRTStar")
plan = group.plan()

# 测试 ImprovedRRT
group.set_planner_id("ImprovedRRT")
plan = group.plan()
```

祝测试顺利！🚀

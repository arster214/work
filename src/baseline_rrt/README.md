# Baseline RRT Package

基础 RRT 和 RRT* 实现，用于与改进算法进行对比。

## 特点

### BasicRRT
- ✅ 使用 KD-Tree (OMPL GNAT) 进行近邻搜索
- ❌ 无目标偏置（纯随机采样）
- ❌ 无其他优化
- 📊 可输出节点数统计

### BasicRRTStar
- ✅ 使用 KD-Tree (OMPL GNAT) 进行近邻搜索
- ✅ 实现 RRT* 的 rewiring 逻辑
- ❌ 无目标偏置（纯随机采样）
- ❌ 无其他优化
- 📊 可输出节点数统计

## 编译

```bash
cd ~/dual_arm_robot_ws
catkin build baseline_rrt
source devel/setup.bash
```

## 使用

### 1. 配置 MoveIt

在你的 MoveIt 配置包中添加 baseline_rrt 规划器：

编辑 `ompl_planning.yaml`:

```yaml
planning_plugin: baseline_rrt/BaselineRRTPlannerManager

planner_configs:
  BasicRRT:
    type: baseline_rrt/BaselineRRTPlannerManager
    range: 0.5
    
  BasicRRTStar:
    type: baseline_rrt/BaselineRRTPlannerManager
    range: 0.5
    rewire_radius: 0.75

r_arm:
  planner_configs:
    - BasicRRT
    - BasicRRTStar
  projection_evaluator: joints(r_shoulder_pan_joint,r_shoulder_lift_joint)
```

### 2. 在代码中使用

```python
import moveit_commander

group = moveit_commander.MoveGroupCommander("r_arm")

# 使用 BasicRRT
group.set_planner_id("BasicRRT")
plan = group.plan()

# 使用 BasicRRTStar
group.set_planner_id("BasicRRTStar")
plan = group.plan()
```

## 与 ImprovedRRT 的对比

| 特性 | BasicRRT | BasicRRTStar | ImprovedRRT (T-RRT+RRT*) |
|------|----------|--------------|--------------------------|
| KD-Tree | ✅ | ✅ | ✅ |
| 目标偏置 | ❌ | ❌ | ✅ |
| T-RRT (代价感知) | ❌ | ❌ | ✅ |
| RRT* (路径优化) | ❌ | ✅ | ✅ |
| Rewiring 优化 | ❌ | 基础版 | 优化版 |

## 用于论文数据采集

这个包专门用于提供 baseline 对比：

1. **BasicRRT** - 展示基础 RRT 的性能
2. **BasicRRTStar** - 展示基础 RRT* 的性能
3. **ImprovedRRT** - 展示你的完整算法的性能

通过对比可以展示：
- T-RRT 代价感知的贡献
- RRT* 路径优化的贡献
- 目标偏置等优化的贡献
- 完整算法的综合优势

## 注意事项

1. **规划时间较长**：由于没有目标偏置，BasicRRT 和 BasicRRTStar 可能需要更长时间才能找到解
2. **建议增加超时**：在数据采集时，建议将超时时间设置为 15-30 秒
3. **节点数统计**：可以通过日志查看生成的节点数

## 许可证

BSD

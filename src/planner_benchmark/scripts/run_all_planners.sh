#!/bin/bash
# 批量运行所有规划器的数据采集
# 每个规划器使用独立的进程，避免 planning pipeline 切换导致的崩溃

set -e

# 配置参数
NUM_RUNS=50
TIMEOUT=30.0
OUTPUT_DIR="$HOME/benchmark_results"
CONFIG_FILE=""

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --runs)
            NUM_RUNS="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

echo "=========================================="
echo "批量数据采集脚本"
echo "=========================================="
echo "重复次数: $NUM_RUNS"
echo "超时时间: $TIMEOUT 秒"
echo "输出目录: $OUTPUT_DIR"
echo "=========================================="
echo ""

# 根据配置自动决定规划器列表
RESOLVED_CONFIG_FILE="$CONFIG_FILE"
if [ -z "$RESOLVED_CONFIG_FILE" ]; then
    RESOLVED_CONFIG_FILE="$(rospack find planner_benchmark)/config/test_poses.yaml"
fi

DETECTED_GROUP=$(python3 - "$RESOLVED_CONFIG_FILE" <<'PY'
import sys
import yaml

config_path = sys.argv[1]
with open(config_path, 'r') as handle:
    config = yaml.safe_load(handle) or {}

def get_arm_map(joint_values):
    if not isinstance(joint_values, dict):
        return None
    left = joint_values.get('l_arm', joint_values.get('left_arm'))
    right = joint_values.get('r_arm', joint_values.get('right_arm'))
    if left is None or right is None:
        return None
    return {'l_arm': list(left), 'r_arm': list(right)}

def is_zero_joint_vector(values):
    return isinstance(values, (list, tuple)) and len(values) > 0 and all(abs(float(v)) <= 1e-9 for v in values)

start_raw = config.get('start_joints', config.get('home_joints'))
goal_pose = config.get('goal_pose') or {}
goal_raw = goal_pose.get('joints') if isinstance(goal_pose, dict) else None

start_arm_map = get_arm_map(start_raw)
goal_arm_map = get_arm_map(goal_raw)
group_name = str(config.get('group_name', 'auto')).strip()

if group_name.lower() == 'auto' and start_arm_map and goal_arm_map:
    left_inactive = is_zero_joint_vector(start_arm_map['l_arm']) and is_zero_joint_vector(goal_arm_map['l_arm'])
    right_inactive = is_zero_joint_vector(start_arm_map['r_arm']) and is_zero_joint_vector(goal_arm_map['r_arm'])
    if left_inactive and not right_inactive:
        print('r_arm')
    elif right_inactive and not left_inactive:
        print('l_arm')
    elif not left_inactive and not right_inactive:
        print('dual_arms')
    else:
        print('invalid')
else:
    print(group_name)
PY
)

if [ "$DETECTED_GROUP" = "dual_arms" ]; then
    PLANNERS=("RRTConnect" "TRRT" "DualArmTRRT")
    echo "检测到双臂配置，将测试: ${PLANNERS[*]}"
elif [ "$DETECTED_GROUP" = "l_arm" ] || [ "$DETECTED_GROUP" = "r_arm" ]; then
    PLANNERS=("RRT" "RRTConnect" "TRRT")
    echo "检测到单臂配置 ($DETECTED_GROUP)，将测试: ${PLANNERS[*]}"
else
    echo "无法从配置中识别单臂/双臂模式，请检查 $RESOLVED_CONFIG_FILE"
    exit 1
fi

# 依次测试每个规划器
for planner in "${PLANNERS[@]}"; do
    echo ""
    echo "=========================================="
    echo "开始测试规划器: $planner"
    echo "=========================================="
    
    # 构建命令
    CMD="rosrun planner_benchmark paper_data_collection.py --planner $planner --runs $NUM_RUNS --timeout $TIMEOUT --output $OUTPUT_DIR"
    
    if [ -n "$CONFIG_FILE" ]; then
        CMD="$CMD --config $CONFIG_FILE"
    fi
    
    echo "执行命令: $CMD"
    echo ""
    
    # 运行测试
    if $CMD; then
        echo ""
        echo "✓ $planner 测试完成"
    else
        echo ""
        echo "✗ $planner 测试失败"
        exit 1
    fi
    
    # 等待一段时间，确保系统稳定
    echo "等待 5 秒后继续下一个规划器..."
    sleep 5
done

echo ""
echo "=========================================="
echo "所有规划器测试完成！"
echo "结果保存在: $OUTPUT_DIR"
echo "=========================================="

# 合并所有 CSV 文件
echo ""
echo "合并所有结果文件..."
MERGED_FILE="$OUTPUT_DIR/benchmark_merged_$(date +%Y%m%d_%H%M%S).csv"

# 获取第一个文件的表头
first_file=$(ls -t "$OUTPUT_DIR"/benchmark_raw_*.csv 2>/dev/null | head -1)
if [ -n "$first_file" ]; then
    head -1 "$first_file" > "$MERGED_FILE"
    
    # 合并所有文件的数据（跳过表头）
    for file in "$OUTPUT_DIR"/benchmark_raw_*.csv; do
        tail -n +2 "$file" >> "$MERGED_FILE"
    done
    
    echo "✓ 合并完成: $MERGED_FILE"
    
    # 统计信息
    total_lines=$(wc -l < "$MERGED_FILE")
    data_lines=$((total_lines - 1))
    echo "总记录数: $data_lines"
else
    echo "✗ 没有找到结果文件"
fi

echo ""
echo "完成！"

#!/bin/bash
# 批量运行所有规划器的数据采集
# 每个规划器使用独立的进程，避免 planning pipeline 切换导致的崩溃

set -e

# 配置参数
NUM_RUNS=20
TIMEOUT=10.0
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

# 规划器列表
PLANNERS=("BasicRRT" "BasicRRTStar" "ImprovedRRT")

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

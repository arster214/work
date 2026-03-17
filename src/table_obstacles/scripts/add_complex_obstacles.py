#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多场景复杂桌面障碍物生成脚本（MoveIt 版）
参考 add_table_obstacles.py，桌面高度保持一致 (TABLE_TOP_Z = 1.20)

场景列表（通过 ROS 参数 ~scenario 选择，默认 1）:
  1 - 平行双墙走廊   : 两道纵墙，正中央留 20 cm Y 向通道
  2 - 多层错位门洞   : 三道横墙，每道留 20 cm 门洞，门洞位置左右交错（折线路径）
  3 - 蜂窝柱阵       : 5 根方柱蜂巢排布，相邻柱间净距约 20 cm
  4 - U 型围栏       : 三面围合，前方 20 cm 开口，内含目标小球
  5 - 蛇形迷宫       : 三道错位挡板，20 cm 开口左右交替，形成 Z 字蛇行路径

用法:
  rosrun table_obstacles add_complex_obstacles.py _scenario:=1
"""

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

# ─────────────────────────────────────────────────────────────────────────────
#  全局参数（与 add_table_obstacles.py 保持一致）
# ─────────────────────────────────────────────────────────────────────────────
FRAME_ID     = "base_link_underpan"

TABLE_POS    = (0.0, -0.9, 0.85)    # 桌体中心 (x, y, z)
TABLE_SIZE   = (1.2, 0.8, 0.7)      # 长(X) × 宽(Y) × 高(Z)
TABLE_TOP_Z  = TABLE_POS[2] + TABLE_SIZE[2] / 2   # = 1.20  桌面 Z 坐标

# 桌面障碍物通用高度（障碍物从桌面向上延伸 0.35 m）
OBS_H = 0.35
OBS_Z = TABLE_TOP_Z + OBS_H / 2     # = 1.375  障碍物中心 Z 坐标

# ─────────────────────────────────────────────────────────────────────────────
#  工具函数
# ─────────────────────────────────────────────────────────────────────────────

def _make_pose(x, y, z):
    p = PoseStamped()
    p.header.frame_id = FRAME_ID
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = float(z)
    p.pose.orientation.w = 1.0
    return p


def add_box(scene, name, x, y, z, size):
    scene.add_box(name, _make_pose(x, y, z), size=size)
    rospy.sleep(0.2)


def add_sphere(scene, name, x, y, z, r):
    scene.add_sphere(name, _make_pose(x, y, z), radius=r)
    rospy.sleep(0.2)


# ─────────────────────────────────────────────────────────────────────────────
#  桌子 + 四柱框架（与原始脚本保持一致）
# ─────────────────────────────────────────────────────────────────────────────

def add_table_and_frame(scene):
    """添加桌子主体及四角柱子、顶部连杆框架"""
    # 桌子
    add_box(scene, "table",
            x=TABLE_POS[0], y=TABLE_POS[1], z=TABLE_POS[2],
            size=TABLE_SIZE)

    # 四角柱子
    half_l    = TABLE_SIZE[0] / 2    # 0.60
    half_w    = TABLE_SIZE[1] / 2    # 0.40
    pillar_h  = 1.5
    pillar_s  = (0.05, 0.05, pillar_h)
    pillars = [
        ("pillar1", +half_l, +half_w + TABLE_POS[1], pillar_h / 2),
        ("pillar2", +half_l, -half_w + TABLE_POS[1], pillar_h / 2),
        ("pillar3", -half_l, +half_w + TABLE_POS[1], pillar_h / 2),
        ("pillar4", -half_l, -half_w + TABLE_POS[1], pillar_h / 2),
    ]
    for n, px, py, pz in pillars:
        add_box(scene, n, x=px, y=py, z=pz, size=pillar_s)

    # 顶部连杆
    rod_t = 0.05
    rod_z = 1.42
    rods = [
        ("rod1", +half_l,              TABLE_POS[1],              rod_z,
         (rod_t, TABLE_SIZE[1], rod_t)),
        ("rod2", -half_l,              TABLE_POS[1],              rod_z,
         (rod_t, TABLE_SIZE[1], rod_t)),
        ("rod3", TABLE_POS[0], +half_w + TABLE_POS[1],            rod_z,
         (TABLE_SIZE[0], rod_t, rod_t)),
        ("rod4", TABLE_POS[0], -half_w + TABLE_POS[1],            rod_z,
         (TABLE_SIZE[0], rod_t, rod_t)),
    ]
    for n, rx, ry, rz, rs in rods:
        add_box(scene, n, x=rx, y=ry, z=rz, size=rs)

    rospy.loginfo("桌子 + 框架已添加")


# =============================================================================
#  场景 1 ── 平行双墙走廊
# =============================================================================
#
#  俯视 (X-Y 平面，Y 向下为靠近机器人):
#
#       X: -0.60        -0.10  +0.10         +0.60
#            ┌─── wall_L ───┐  通道  ┌── wall_R ──┐
#            │(0.50×0.65)   │  20cm  │(0.50×0.65) │
#            └──────────────┘        └────────────┘
#
#  两道纵墙沿 Y 延伸，正中央留 20 cm（X∈[-0.10,+0.10]）通道，
#  双臂可沿 Y 方向穿越通道进入桌面深处。
#  通道内放置两个目标球，验证穿越能力。
#
def scenario1_parallel_walls(scene):
    rospy.loginfo("[场景1] 平行双墙走廊 — 添加中...")

    # 左墙：X 从 -0.60 到 -0.10，中心 X = -0.35，宽 0.50 m
    add_box(scene, "wall_L",
            x=-0.35, y=TABLE_POS[1], z=OBS_Z,
            size=(0.50, 0.65, OBS_H))

    # 右墙：X 从 +0.10 到 +0.60，中心 X = +0.35，宽 0.50 m
    add_box(scene, "wall_R",
            x=+0.35, y=TABLE_POS[1], z=OBS_Z,
            size=(0.50, 0.65, OBS_H))

    # 通道内目标球（半径 0.05，放在桌面上方 5 cm）
    ball_z = TABLE_TOP_Z + 0.05
    add_sphere(scene, "ball_c1", x=0.0, y=-0.72, z=ball_z, r=0.05)
    add_sphere(scene, "ball_c2", x=0.0, y=-1.08, z=ball_z, r=0.05)

    rospy.loginfo("[场景1] 完成  通道宽 20 cm，中心 X=0")


# =============================================================================
#  场景 2 ── 多层错位门洞
# =============================================================================
#
#  俯视 (X-Y 平面):
#
#   Y=-0.72: ┌────────────────────────────────┐□20cm┐  门洞在 X=+0.40 右侧
#   Y=-0.90: ┌20cm□┌──────────────────────────────┐  门洞在 X=0    中央
#   Y=-1.08: ┌────────────────────────────────┐□20cm┐  门洞在 X=+0.40 右侧
#             (同第一道，形成 Z 字折线路径)
#
#  三道横墙（沿 X 延伸），每道留 20 cm 宽门洞；
#  门洞位置依次为：右 → 中 → 右，迫使手臂走折线路径。
#
def scenario2_staggered_gates(scene):
    rospy.loginfo("[场景2] 多层错位门洞 — 添加中...")

    wall_t = 0.05         # 墙厚 (Y 向)
    opening = 0.20        # 门洞宽 = 20 cm

    # (名称前缀, 墙的 Y 坐标, 门洞中心 X)
    gates = [
        ("a", -0.72, +0.40),   # 右侧开门
        ("b", -0.90,  0.00),   # 中央开门
        ("c", -1.08, +0.40),   # 右侧开门（同第一道，Z 字回折）
    ]

    for sfx, gy, cx in gates:
        lo = cx - opening / 2    # 门洞左边界 X
        hi = cx + opening / 2    # 门洞右边界 X

        # 门洞左侧墙段
        x_left_end = -0.60
        if lo > x_left_end:
            lw = lo - x_left_end
            add_box(scene, f"gate_{sfx}_L",
                    x=x_left_end + lw / 2, y=gy, z=OBS_Z,
                    size=(lw, wall_t, OBS_H))

        # 门洞右侧墙段
        x_right_end = +0.60
        if hi < x_right_end:
            rw = x_right_end - hi
            add_box(scene, f"gate_{sfx}_R",
                    x=hi + rw / 2, y=gy, z=OBS_Z,
                    size=(rw, wall_t, OBS_H))

    # 每道门洞后方各放一个目标球
    ball_z = TABLE_TOP_Z + 0.05
    ball_positions = [
        ("ball_g1", +0.40, -0.72 - 0.10),   # 第一道门后方
        ("ball_g2",  0.00, -0.90 - 0.10),   # 第二道门后方
        ("ball_g3", +0.40, -1.08 - 0.05),   # 第三道门后方（贴近桌边）
    ]
    for n, bx, by in ball_positions:
        add_sphere(scene, n, x=bx, y=by, z=ball_z, r=0.04)

    rospy.loginfo("[场景2] 完成  3 道门洞各宽 20 cm，门洞 X 依次: +0.40 / 0 / +0.40")


# =============================================================================
#  场景 3 ── 蜂窝柱阵
# =============================================================================
#
#  俯视 (X-Y 平面):
#
#   Y=-0.75: ●         ●         ●      (x = -0.32, 0.00, +0.32)
#   Y=-1.05:     ●         ●            (x = -0.16, +0.16)
#
#  5 根 12×12 cm 方柱按蜂巢形排布，相邻柱间净距：
#   Row 1 相邻: 0.32×2 - 0.12 = 0.52 ÷ 2?
#   实际净距 = |0.00 - (-0.32)| - 0.12 = 0.20 m = 20 cm ✓
#   Row 2 相邻:  0.16×2      - 0.12 = 0.20 m = 20 cm ✓
#
def scenario3_honeycomb_pillars(scene):
    rospy.loginfo("[场景3] 蜂窝柱阵 — 添加中...")

    col_s = (0.12, 0.12, OBS_H)    # 柱子截面 12×12 cm

    pillars = [
        # Row 1（前排，3 根）
        ("col_r1_l", -0.32, -0.75),
        ("col_r1_c",  0.00, -0.75),
        ("col_r1_r", +0.32, -0.75),
        # Row 2（后排，2 根，错位排列）
        ("col_r2_l", -0.16, -1.05),
        ("col_r2_r", +0.16, -1.05),
    ]

    for name, px, py in pillars:
        add_box(scene, name, x=px, y=py, z=OBS_Z, size=col_s)

    # 目标球散落在柱阵间隙深处
    ball_z = TABLE_TOP_Z + 0.05
    targets = [
        ("ball_h1",  0.00, -0.90, ball_z, 0.04),   # 前排两柱中间
        ("ball_h2", -0.24, -1.15, ball_z, 0.04),   # 后排左侧外
        ("ball_h3", +0.24, -1.15, ball_z, 0.04),   # 后排右侧外
    ]
    for n, x, y, z, r in targets:
        add_sphere(scene, n, x=x, y=y, z=z, r=r)

    rospy.loginfo("[场景3] 完成  5 柱蜂巢阵，列间净距 20 cm")


# =============================================================================
#  场景 4 ── U 型围栏
# =============================================================================
#
#  俯视 (X-Y 平面):
#
#    Y=-0.72 侧（靠机器人）: 开口 20 cm（X∈[-0.10,+0.10]）
#         ↕ side_L                ↕ side_R
#    Y=-1.225：────────── back ──────────
#
#  三面围合，前方留 20 cm 开口。
#  围栏内放目标球，双臂需从开口进入后取球。
#
def scenario4_u_corral(scene):
    rospy.loginfo("[场景4] U 型围栏 — 添加中...")

    wall_t      = 0.05
    inner_half  = 0.10    # 开口半宽 = 10 cm → 开口总宽 20 cm
    side_cx     = inner_half + wall_t / 2    # 侧墙中心 X = 0.125

    # 侧墙 Y 范围：从 -0.72 到 -1.20（长 0.48 m，中心 Y = -0.96）
    side_y_center = -0.96
    side_y_len    = 0.48

    # 左侧墙
    add_box(scene, "u_side_L",
            x=-side_cx, y=side_y_center, z=OBS_Z,
            size=(wall_t, side_y_len, OBS_H))

    # 右侧墙
    add_box(scene, "u_side_R",
            x=+side_cx, y=side_y_center, z=OBS_Z,
            size=(wall_t, side_y_len, OBS_H))

    # 后墙（连接两侧墙外沿：X 宽度 = side_cx×2 = 0.25 m）
    back_y  = -1.20 - wall_t / 2    # 后墙中心 Y
    back_xw = side_cx * 2           # = 0.25 m
    add_box(scene, "u_back",
            x=0.0, y=back_y, z=OBS_Z,
            size=(back_xw, wall_t, OBS_H))

    # 围栏内目标球
    add_sphere(scene, "ball_u_target",
               x=0.0, y=-1.00, z=TABLE_TOP_Z + 0.05, r=0.05)

    rospy.loginfo("[场景4] 完成  U 型开口宽 20 cm（X∈[-0.10,+0.10]），开口朝 Y=-0.72 侧")


# =============================================================================
#  场景 5 ── 蛇形迷宫
# =============================================================================
#
#  俯视 (X-Y 平面):
#
#   Y=-0.72: ┌──────── baffle_1 (X:-0.60→+0.40) ───┐  □ 开口 20cm 在右
#   Y=-0.90: □ 开口 20cm 在左  ┌── baffle_2 (X:-0.40→+0.60) ──┐
#   Y=-1.08: ┌──────── baffle_3 (X:-0.60→+0.40) ───┐  □ 开口 20cm 在右
#
#  三道横向挡板，开口在左右两侧交替出现（右→左→右），
#  机器人必须走蛇形 Z 字路线才能穿越障碍区。
#
def scenario5_snake_maze(scene):
    rospy.loginfo("[场景5] 蛇形迷宫 — 添加中...")

    wall_t = 0.05
    # 挡板宽 1.00 m，中心偏左 → 右侧留 20 cm 开口（+0.40 到 +0.60）
    # 挡板宽 1.00 m，中心偏右 → 左侧留 20 cm 开口（-0.60 到 -0.40）
    baffles = [
        #  名称       中心 X   中心 Y   宽度    开口侧说明
        ("baffle_1", -0.10,  -0.72,  1.00),  # 开口在右 X∈[+0.40,+0.60]
        ("baffle_2", +0.10,  -0.90,  1.00),  # 开口在左 X∈[-0.60,-0.40]
        ("baffle_3", -0.10,  -1.08,  1.00),  # 开口在右 X∈[+0.40,+0.60]
    ]

    for name, bx, by, bw in baffles:
        add_box(scene, name,
                x=bx, y=by, z=OBS_Z,
                size=(bw, wall_t, OBS_H))

    # 每道挡板的开口旁放置目标球，验证蛇形路径可达性
    ball_z = TABLE_TOP_Z + 0.05
    targets = [
        ("ball_s1", +0.50, -0.82, ball_z, 0.04),   # 第1道右侧开口后方
        ("ball_s2", -0.50, -1.00, ball_z, 0.04),   # 第2道左侧开口后方
        ("ball_s3", +0.50, -1.16, ball_z, 0.04),   # 第3道右侧开口后方（贴近桌边）
    ]
    for n, x, y, z, r in targets:
        add_sphere(scene, n, x=x, y=y, z=z, r=r)

    rospy.loginfo("[场景5] 完成  3 道挡板开口宽 20 cm，左右交替（右→左→右）")


# =============================================================================
#  场景注册表 & 主入口
# =============================================================================

SCENARIOS = {
    1: ("平行双墙走廊", scenario1_parallel_walls),
    2: ("多层错位门洞", scenario2_staggered_gates),
    3: ("蜂窝柱阵",     scenario3_honeycomb_pillars),
    4: ("U 型围栏",     scenario4_u_corral),
    5: ("蛇形迷宫",     scenario5_snake_maze),
}


def main():
    rospy.init_node("add_complex_obstacles")

    scenario_id = int(rospy.get_param("~scenario", 1))

    if scenario_id not in SCENARIOS:
        rospy.logerr(
            f"无效场景编号 {scenario_id}，可选值: {sorted(SCENARIOS.keys())}"
        )
        return

    name, fn = SCENARIOS[scenario_id]
    rospy.loginfo(f"{'='*50}")
    rospy.loginfo(f"场景 {scenario_id}: {name}")
    rospy.loginfo(f"桌面高度 TABLE_TOP_Z = {TABLE_TOP_Z:.3f} m")
    rospy.loginfo(f"障碍物中心 Z  OBS_Z  = {OBS_Z:.3f} m")
    rospy.loginfo(f"{'='*50}")

    rospy.loginfo("等待 MoveIt 规划场景接口初始化...")
    rospy.sleep(3)
    scene = PlanningSceneInterface()
    rospy.sleep(3)

    rospy.loginfo("添加桌子 + 框架...")
    add_table_and_frame(scene)

    rospy.loginfo(f"添加场景 {scenario_id} 障碍物...")
    fn(scene)

    rospy.loginfo("所有障碍物已成功添加到 MoveIt 规划场景，节点保持运行...")
    rospy.spin()


if __name__ == "__main__":
    main()

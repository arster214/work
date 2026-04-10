# EnvironmentInfo.msg
# 提供全局环境的摘要，发送给热力图生成器或规划器

# 桌面的高度，或者是主要参考地面的Z轴坐标
float32 table_z

# 环境中所有障碍物的包围盒列表
BoundingBox[] obstacles

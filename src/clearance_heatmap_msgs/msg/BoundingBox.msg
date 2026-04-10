# BoundingBox.msg
# 定义单个障碍物轴对齐包围盒 (AABB) 属性，用于热力图与规划查询

# 包围盒中心点的三维坐标 (x, y, z)
geometry_msgs/Point center

# 包围盒的三个维度尺寸 (length_x, length_y, length_z)
geometry_msgs/Vector3 size

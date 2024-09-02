# 非标准形式---------------------------------

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # 生成椭球参数
# center = np.array([1, 2, 3])  # 中心点坐标
# radii = np.array([2, 3, 4])   # 长短半轴长度
# rotation_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # 方向矩阵

# # 生成椭球上的点
# phi = np.linspace(0, 2 * np.pi, 100)
# theta = np.linspace(0, np.pi, 50)
# x = radii[0] * np.outer(np.cos(phi), np.sin(theta))
# y = radii[1] * np.outer(np.sin(phi), np.sin(theta))
# z = radii[2] * np.outer(np.ones_like(phi), np.cos(theta))

# # 应用旋转矩阵
# for i in range(len(x)):
#     for j in range(len(x[i])):
#         [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation_matrix)

# # 平移椭球到指定位置
# x += center[0]
# y += center[1]
# z += center[2]

# # 可视化
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot_wireframe(x, y, z, color='b', alpha=0.3)  # 绘制网格
# ax.scatter(center[0], center[1], center[2], color='r', s=100)  # 标记中心点
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.show()
# ---------------------------------------------

# 标准形式
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 定义椭球的参数
center = np.array([5, 5, 5])  # 中心点坐标
A = np.diag([100, 100, 100])  # 半径矩阵，对角元素为长轴、短轴、短轴的平方

# 生成椭球上的点
phi = np.linspace(0, 2 * np.pi, 100)
theta = np.linspace(0, np.pi, 50)
x = center[0] + np.outer(np.cos(phi), np.sin(theta)) * np.sqrt(A[0, 0])
y = center[1] + np.outer(np.sin(phi), np.sin(theta)) * np.sqrt(A[1, 1])
z = center[2] + np.outer(np.ones_like(phi), np.cos(theta)) * np.sqrt(A[2, 2])

# 可视化
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_wireframe(x, y, z, color='b', alpha=0.3)  # 绘制网格
ax.scatter(center[0], center[1], center[2], color='r', s=100)  # 标记中心点
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 固定坐标轴长度
ax.set_xlim(center[0] - 15, center[0] + 15)
ax.set_ylim(center[1] - 15, center[1] + 15)
ax.set_zlim(center[2] - 15, center[2] + 15)

plt.show()




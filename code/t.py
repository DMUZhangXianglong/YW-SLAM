'''
* @Author: DMU zhangxianglong
* @Date: 2024-08-12 18:47:28
* @LastEditTime: 2024-08-14 14:48:59
* @LastEditors: DMU zhangxianglong
* @FilePath: /code/t.py
* @Description: 
'''
import matplotlib.pyplot as plt

# 定义两点的坐标
p1 = (1, 2)
p2 = (4, 5)

# 创建一个图形
plt.figure()

# 绘制两点之间的连线
plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-', lw=2)  # 'k-' 表示黑色实线

# 在连线上添加箭头，表示从p1到p2的方向
plt.arrow(p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1], head_width=0.1, head_length=0.1, fc='blue', ec='red')

# 设置坐标轴的范围
plt.xlim(min(p1[0], p2[0]) - 1, max(p1[0], p2[0]) + 1)
plt.ylim(min(p1[1], p2[1]) - 1, max(p1[1], p2[1]) + 1)

# 添加网格
plt.grid(True)

# 显示图形
plt.show()



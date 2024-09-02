'''
* @Author: DMU zhangxianglong
* @Date: 2024-08-12 18:47:28
* @LastEditTime: 2024-08-14 15:44:50
* @LastEditors: DMU zhangxianglong
* @FilePath: /code/linearSystem.py
* @Description: 
'''
'''
* @Author: DMU zhangxianglong
* @Date: 2024-08-12 10:20:43
* @LastEditTime: 2024-08-12 23:07:23
* @LastEditors: DMU zhangxianglong
* @FilePath: /code/linearSystem.py
* @Description: 线性二维系统的退化
'''
import numpy as np
import matplotlib.pyplot as plt


# 获取直线交点
def getPoint(m1, b1, m2, b2):
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    return x, y

# 求解最小二乘问题 f = argmin_x ||Ax - b||^2
def solve(A, b):
    # ATA 和 ATb
    ATA = A.T @ A
    ATb = A.T @ b
    return np.linalg.solve(ATA, ATb)

# 计算两点之间的距离
def getDistance(p1, p2):
    temp = p1 - p2
    d = np.sqrt(temp[0]*temp[0] + temp[1]*temp[1])
    return d


# 给定3条直线参数
a = [1, -1, 0]
b = [-1, 8, 2.5]

# 扰动移动的距离
delta_d = -1.5

# 扰动直线的斜率
insert_a = 0.16
# 截距
insert_b = 3.16666667 + delta_d

# insert_b = 2.49166667 + delta_d

# a = [1, -1, 1, -1]
# b = [0, 0, -2, 2]

# a = [1, -1, 1, -1]
# b = [0, 0, -2, 2]


# 给定 x 的值
x = np.linspace(2.5, 6.5, 400)
# 计算 y 的值 并计算交点坐标
y = []
x_p = []
y_p = []
for i in range(len(a)):
    # 计算 y 的值
    y.append(a[i] * x + b[i])
    # 计算交点坐标
    if i < (len(a)-1) :
        x_i, y_i = getPoint(a[i], b[i], a[i+1], b[i+1])
        x_p.append(x_i)
        y_p.append(y_i)
    else:
        x_i, y_i = getPoint(a[0], b[0], a[i], b[i])
        x_p.append(x_i)
        y_p.append(y_i)

# 填充原始解的区域
plt.fill(x_p, y_p, color='gray', alpha=0.5, label='Original Solution Area')

# 新的y
y.append(insert_a * x + insert_b)

# 画出 3 条直线
plt.plot(x, y[0], label='y1', color='black')
plt.plot(x, y[1], label='y2', color='black')
plt.plot(x, y[2], label='y3', color='black')
# 新的约束 扰动
plt.plot(x, y[3], label='y4', color='red')


# 定义三个原始不等式约束
A = np.array([[1, -1], [-1, -1], [0, -1]])
b = np.array([[1], [-8], [-2.5]])

# 定义新的约束
c = np.array([[insert_a, -1]])
d = np.array([[-insert_b]])

# 新的 A 和 b
new_A = np.vstack((A, c))
new_b = np.vstack((b, d))

# new_A = np.delete(A, 1, axis=0)
# new_b = np.delete(b, 1, axis=0)

# new_A = A
# new_b = np.array([[1], [-5], [13]])

# A = np.array([[1, 2], [3, 4], [5, 6]])
# b = np.array([1, 2, 3])

# 求解该最小二乘问题 f = argmin_x ||Ax - b||^2 原始解
original_x_optimal = solve(A, b)
print("原始最优的x是:", original_x_optimal)
# 求解该最小二乘问题 f = argmin_x ||Ax - b||^2 加入扰动的解
new_x_optimal = solve(new_A, new_b)
print("新的最优的x是:", new_x_optimal)

# 计算解 x 的移动距离
delta_x = getDistance(original_x_optimal, new_x_optimal)
print('delta_x', delta_x[0])
plt.plot([original_x_optimal[0], new_x_optimal[0]], [original_x_optimal[1], new_x_optimal[1]],'--', lw=1.0, color="green", label=f'$\delta x$: {delta_x[0]}') 


# 画出原始解
plt.plot(original_x_optimal[0], original_x_optimal[1], 'ro', label=f'Original Solution: {original_x_optimal[0][0],original_x_optimal[1][0]}')
# 画出新的解
plt.plot(new_x_optimal[0], new_x_optimal[1], 'x', label=f'New Solution: {new_x_optimal[0][0],new_x_optimal[1][0]}')

# plt.xlim((0, 8))
# plt.ylim((0, 6))
plt.xticks(np.arange(-3,10, 1))
plt.yticks(np.arange(-3,10, 1))
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')
plt.axhline(0, color='blue',linewidth=1.5)
plt.axvline(0, color='blue',linewidth=1.5)
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.title('Linear System')
plt.grid(True)
plt.show()











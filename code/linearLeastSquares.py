'''
* @Author: DMU zhangxianglong
* @Date: 2024-08-03 16:11:28
* @LastEditTime: 2024-08-12 09:40:51
* @LastEditors: DMU zhangxianglong
* @FilePath: /code/linearLeastSquares.py
* @Description: 线性最小二乘的一个例子
'''

import numpy as  np

# 随机数种子
np.random.seed(0)

# true_beta
true_beta = np.array([1.5, 2.6, 5.2]).reshape(3,1)

# X 
X = np.random.rand(3,3)


# Y
Y = X @ true_beta + 0.1 * np.random.rand(3,1)

# Q R 分解
Q, R = np.linalg.qr(X)
# Q^T * Y
QT_Y = Q.T @ Y

# est_beta 
est_beta = np.linalg.solve(R, QT_Y)

print("true beta", true_beta)
print("est beta", est_beta)
'''
* @Author: DMU zhangxianglong
* @Date: 2024-07-31 15:02:19
* @LastEditTime: 2024-07-31 17:37:11
* @LastEditors: DMU zhangxianglong
* @FilePath: /code/gaussNewton.py
* @Description: 
'''
import numpy as np

def main():
    # 真实参数
    ar = 1.0
    br = 2.0 
    cr = 1.0

    # 估计参数值
    ae = 2.0
    be = -1.0
    ce = 5.0

    # 数据个数
    N = 100

    # 噪声的方差
    w_sigma = 1.0 

    x_data = []
    y_data = []
    
    # 生成数据
    for i in range(N):
        x = i / 100.0
        x_data.append(x)
        y_data.append((np.exp(ar * x * x + br * x  +cr ) + np.random.normal(0, w_sigma * w_sigma)))
    
    # Gauss-Newton
    interations = 100
    cost = 0
    last_cost = 0

    for inter in range(interations):
        H = np.zeros((3,3)) # J^T * J
        b = np.zeros((3,3)) # bias
        cost = 0
        for i in range(N):
            xi = x_data[i] 
            yi = y_data[i]
            error = yi - np.exp(ae * xi * xi + be * xi  +ce) # 误差
            J = np.zeros(3) # 雅可比矩阵
            J[0] = -xi * xi * np.exp(ae * xi * xi + be * xi + ce)
            J[1] = -xi * np.exp(ae * xi * xi + be * xi + ce)
            J[2] = -np.exp(ae * xi * xi + be * xi + ce)
            

main()




    

'''
* @Author: DMU zhangxianglong
* @Date: 2024-09-09 15:52:47
* @LastEditTime: 2024-09-11 21:19:35
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/ros2_ws/lidar_2d/test/my_test.py
* @Description: 
'''


import cv2
import numpy as np
import sophuspy as sp

# print(dir(sp.SE2()))
pose = sp.SE2()
vec2d = np.array([1, 2])

pw = pose * vec2d

# print(dir(sp.SE2))
# print()
# print(dir(pose))
print(pw[0])



# 创建一个李代数元素（se2 元素：平移和旋转）
# xi = np.array([0.1, 0.2, 0.3])  # 示例向量 (vx, vy, theta)

# # 使用 SE2 的 exp() 方法
# se2_exp = sp.SE2.exp(xi)  # 将李代数元素映射到李群

# print(se2_exp)
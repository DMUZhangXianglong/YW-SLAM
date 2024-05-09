'''
Author: DMU zhangxianglong
Date: 2024-05-09 10:17:57
* @LastEditTime: 2024-05-10 00:51:49
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/imu_process/utils.py
Description: 
'''
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from scipy.linalg import expm

# 读取文件，去除第一个字段不是 "IMU" 的行，并保存到新文件中
def process_file(input_file, output_file):
    '''
    * @description: 
    * @param {*} input_file:
    * @param {*} output_file:
    * @return {*}
    '''
    with open(input_file, "r") as f_in:
        lines = f_in.readlines()

    filtered_lines = [line.replace("IMU", "").strip() + "\n" for line in lines if line.strip().split()[0] == "IMU"]

    with open(output_file, "w") as f_out:
        f_out.writelines(filtered_lines)

# 计算李群李代数
def so3ToSO3(omga):
    exp_omga = np.array([[0.0, -omga[2][0], omga[1][0]],
                         [omga[2][0], 0.0, -omga[0][0]],
                         [-omga[1][0], omga[0][0], 0.0]])
    
    return expm(exp_omga)

# 显示类
class Viewer():
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.x_data = []
        self.y_data = []
        self.z_data = []
        self.sc = self.ax.scatter(self.x_data, self.y_data, self.z_data)
        self.line = None
    
    def update3D(self, p):
        # 更新数据
        self.x_data.append(p[0][0])
        self.y_data.append(p[1][0])
        self.z_data.append(p[2][0])
        
        if self.line is not None:
            self.line.pop(0).remove()  # 删除之前的轨迹线
        
        self.line = self.ax.plot(self.x_data, self.y_data, self.z_data, color='blue')  # 绘制新的轨迹线
        
        # 重新绘制
        plt.draw()
        plt.pause(0.001)  # 暂停一小段时间以便图形更新

    def update2D(self, p):
        pass
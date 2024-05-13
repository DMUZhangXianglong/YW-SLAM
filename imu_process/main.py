'''
Author: DMU zhangxianglong
Date: 2024-05-09 10:17:57
* @LastEditTime: 2024-05-13 15:38:23
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/imu_process/main.py
Description: 
'''

from Imu import IMU
from ImuIntegration import ImuIntegration
from utils import Viewer
import numpy as np
from tqdm import tqdm
import time
import matplotlib.pyplot as plt
from utils import process_file 



# 数据预处理
# process_file("./imu_process/imu_11.txt", "./imu_process/imu11_out.txt")

imu_data_path = "./imu_process/imu_out.txt"
imu_data = np.loadtxt("./imu_process/imu_out.txt")
count_lins = imu_data.shape[0]

def main():
    if imu_data.size != 0:
        # 初始化viewer
        # viewer = Viewer()

        # 初始化IMU
        gravity = np.array([0.0, 0.0, -9.8])
        init_bg = np.array([0.000224886, -7.61038e-05, -0.000742259])
        init_ba = np.array([-0.165205, 0.0926887, 0.0058049])
        imu = IMU(imu_data, gravity, init_bg, init_ba)
        
        # 初始化积分器
        imu_integration = ImuIntegration()
        VX = []
        VY = []
        VZ = []
        
        # 主循序
        print("开始主循环")
        for line in tqdm(range(count_lins)):
            imu.updateData(line)
            imu_integration.integration(imu)
            
            VX.append(imu_integration.v[0][0])
            VY.append(imu_integration.v[1][0])
            VZ.append(imu_integration.v[2][0])
            
        # 画图
        plt.figure()
        # 绘制图像
        plt.plot(range(len(VX)), VX, label='vx')
        plt.plot(range(len(VY)), VY, label='vy')
        plt.plot(range(len(VZ)), VZ, label='vz')
        
        # 添加标签和标题
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Sine Function')

        # 添加图例
        plt.legend()

        # 显示图表
        plt.show()
        
    else:
        print("Please check the IMU data file.")

    
main()

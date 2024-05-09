'''
Author: DMU zhangxianglong
Date: 2024-05-09 10:17:57
* @LastEditTime: 2024-05-09 17:33:24
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/imu_process/main.py
Description: 
'''

from Imu import IMU
from ImuIntegration import ImuIntegration
import numpy as np
import time


# 数据预处理
# utils.process_file("./imu_process/imu.txt", "./imu_process/imu_out.txt")

imu_data_path = "./imu_process/imu_out.txt"
imu_data = np.loadtxt("./imu_process/imu_out.txt")
count_lins = imu_data.shape[0]

def main():
    if imu_data.size != 0:
        # 初始化IMU
        gravity = np.array([0.0, 0.0, -9.8])
        init_bg = np.array([00.000224886, -7.61038e-05, -0.000742259])
        init_ba = np.array([-0.165205, 0.0926887, 0.0058049])
        imu = IMU(imu_data, gravity, init_bg, init_ba)
        
        # 初始化积分器
        imu_integration = ImuIntegration()
        
        # 主循序
        print("开始主循环")
        for line in range(count_lins):
            imu.updateData(line)
            imu_integration.integration(imu)
            time.sleep(0.5)
    else:
        print("Please check the IMU data file.")

    
main()
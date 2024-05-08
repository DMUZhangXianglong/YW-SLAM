'''
@Author: DMU zhangxianglong
@Date: 2024-05-08 15:01:35
@LastEditTime: 2024-05-08 17:45:45
@LastEditors: DMU zhangxianglong
@FilePath: \YW-SLAM\imu_process\Imu.py
@Description: 
'''

import numpy as np


class IMU():
    def __init__(self, data):
        self.data = data
        self.gyroscope = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.time = 0.0
    
    def updateData(self, line):
        imu_data = self.data[line]
        self.time = imu_data[0]
        self.gyroscope[0] = imu_data[1]
        self.gyroscope[1] = imu_data[2]
        self.gyroscope[2] = imu_data[3]
        self.acceleration[0] = imu_data[4]
        self.acceleration[1] = imu_data[5]
        self.acceleration[2] = imu_data[6]





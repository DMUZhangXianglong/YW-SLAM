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
    def __init__(self, file_path):
        self.file_path = file_path
        self.gyroscope = np.zeros((3, 1))
        self.acceleration = np.zeros((3, 1))
        self.time = 0.0
    
    def getImuData(self):
        print(self.file_path)
        with open(self.file_path, "r", encoding="utf-8") as file:
            for line in file:
                line = line.strip()  # 去除行首行尾的空白字符
                if line.startswith("IMU"):
                    elements = line.split()  # 使用空格分割每一行的元素
                    print(elements)        


imu = IMU("./imu_process/imu.txt")
imu.getImuData()
    
'''
* @Author: DMU zhangxianglong
* @Date: 2024-05-08 15:01:35
* @LastEditTime: 2024-05-09 17:32:34
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/imu_process/Imu.py
* @Description: 组织imu数据
'''


import numpy as np


class IMU():
    def __init__(self, data, gravity, init_bg, init_ba):
        '''
        * @description: 
        * @param {*} self:
        * @param {*} data:IMU数据
        * @param {*} gravity:重力加速度
        * @param {*} init_bg:陀螺仪偏置
        * @param {*} init_ba:加速度计偏置
        * @return {*}
        '''
        self.data = data
        self.gyroscope = np.zeros(3)
        self.acceleration = np.zeros(3)
        self.timestamp = 0.0

        self.gravity = gravity.reshape((3,1))
        self.init_bg = init_bg.reshape((3,1))
        self.init_ba = init_ba.reshape((3,1))

    
    def updateData(self, line):
        imu_data = self.data[line]
        self.timestamp = imu_data[0]
        self.gyroscope[0] = imu_data[1]
        self.gyroscope[1]= imu_data[2]
        self.gyroscope[2] = imu_data[3]
        self.acceleration[0] = imu_data[4]
        self.acceleration[1] = imu_data[5]
        self.acceleration[2] = imu_data[6]
        # 改为3*1向量
        self.gyroscope = self.gyroscope.reshape((3,1))
        self.acceleration = self.acceleration.reshape((3,1))
        





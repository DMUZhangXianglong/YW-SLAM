'''
* @Author: DMU zhangxianglong
* @Date: 2024-05-08 14:39:57
* @LastEditTime: 2024-05-09 17:30:47
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/imu_process/ImuIntegration.py
* @Description: 实现imu积分
'''
import numpy as np
from Imu import IMU
from utils import so3ToSO3

class ImuIntegration():
    def __init__(self):
        self.timestamp = 0.0
        # 定义旋转 速度 位移
        self.R = np.identity(3)
        self.v = np.zeros((3,1))
        self.p = np.zeros((3,1))


    def integration(self, imu:IMU):
        '''
        * @description: 
        * @param {*} self:
        * @param {IMU} imu:IMU类实例
        * @return {*}
        '''
        dt = imu.timestamp - self.timestamp
        if dt > 0.0 and dt < 0.1:
            # 位移
            self.p = self.p + self.v * dt  + (0.5 * np.dot(self.R, (imu.acceleration - imu.init_ba)) * dt * dt) + 0.5 * imu.gravity * dt * dt
            
            # 速度
            self.v = self.v  + (np.dot(self.R, (imu.acceleration - imu.init_ba)) * dt) + imu.gravity * dt
            
            print("acceleration:", imu.acceleration.shape)
            print("init_ba:", imu.init_ba.shape)
            print("temp:", (imu.acceleration - imu.init_ba).shape)
            print("v:", self.v.shape)
            # 旋转
            omga = (imu.acceleration - imu.init_bg) * dt
            self.R = np.dot(self.R, so3ToSO3(omga))
        self.timestamp = imu.timestamp 
        
        # print("position:", self.p.shape)
        # print("R:", self.R.shape)
        # print("v:", self.v.shape,"\n")
        
        



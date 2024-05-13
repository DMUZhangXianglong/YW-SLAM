'''
* @Author: DMU zhangxianglong
* @Date: 2024-05-08 14:39:57
* @LastEditTime: 2024-05-13 15:38:28
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
            self.p = self.p + self.v * dt  + (0.5 * np.dot(self.R, ((imu.acceleration - imu.init_ba)* dt * dt)))  + (0.5 * imu.gravity * dt * dt)
            # print(np.dot(self.R, ((imu.acceleration - imu.init_ba)* dt *dt)))
            # print(np.dot(self.R, (imu.acceleration - imu.init_ba)) * dt *dt )
            
            # self.p = self.p + self.v * dt  + 0.5 * (self.R @ (imu.acceleration - imu.init_ba)) * dt * dt + 0.5 * imu.gravity * dt * dt

            # 速度
            self.v = self.v  + np.dot(self.R, (imu.acceleration - imu.init_ba)) * dt + imu.gravity * dt
            # self.v = self.v  + self.R @ (imu.acceleration - imu.init_ba) * dt  + imu.gravity * dt

            # 旋转
            omga = (imu.gyroscope - imu.init_bg) * dt
            self.R = np.dot(self.R, so3ToSO3(omga))
            # print("result:", so3ToSO3(omga) , "\n")
       
        self.timestamp = imu.timestamp 
        
        # print("p.x:", self.p[0][0],"\n")
        # print("R:", self.R.shape)
        # print("v:", self.v.shape,"\n")
        
        



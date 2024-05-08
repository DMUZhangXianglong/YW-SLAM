'''
Author: DMU zhangxianglong
Date: 2024-05-08 14:39:57
@LastEditTime: 2024-05-08 16:26:28
@LastEditors: DMU zhangxianglong
@FilePath: \YW-SLAM\imu_process\ImuIntegration.py
Description: 
'''

class ImuIntegration():
    
    def __init__(self, gravity, init_bg, init_ba):
        '''
        @description: 
        @param {*} self:
        @param {*} gravity:重力加速度初始值
        @param {*} init_bg:陀螺仪偏置初始值
        @param {*} init_ba:加速度计偏置初始值
        @return {*}
        '''
        self.gravity = gravity  
        self.initbg = init_bg
        self.initba = init_ba
    
    def getImuData():
        pass



'''
Author: DMU zhangxianglong
Date: 2024-05-09 10:17:57
* @LastEditTime: 2024-05-10 02:20:01
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
process_file("./imu_process/imu_11.txt", "./imu_process/imu11_out.txt")

imu_data_path = "./imu_process/imu11_out.txt"
imu_data = np.loadtxt("./imu_process/imu11_out.txt")
count_lins = imu_data.shape[0]

def main():
    if imu_data.size != 0:
        # 初始化viewer
        # viewer = Viewer()

        # 初始化IMU
        gravity = np.array([0.0, 0.0, -9.8])
        init_bg = np.array([00.000224886, -7.61038e-05, -0.000742259])
        init_ba = np.array([-0.165205, 0.0926887, 0.0058049])
        imu = IMU(imu_data, gravity, init_bg, init_ba)
        
        # 初始化积分器
        imu_integration = ImuIntegration()
        
        # 主循序
        print("开始主循环")
        x = 0.0
        y = 0.0
        z = 0.0
        vx = 0.0
        vy = 0.0
        vz = 0.0
        lx = []
        ly = []
        lz = []

        lvx = []
        lvy = []
        lvz = []
        for line in tqdm(range(count_lins)):
            imu.updateData(line)
            imu_integration.integration(imu)
            x = imu_integration.p[0][0]
            y = imu_integration.p[1][0]
            z = imu_integration.p[2][0]
            lx.append(x)
            ly.append(y)

            vx = imu_integration.v[0][0]
            vy = imu_integration.v[1][0]
            vz = imu_integration.v[2][0]
            lvx.append(vx)
            lvy.append(vy)
            lvz.append(vz)
            # time.sleep(2)
            # viewer.update3D(imu_integration.p)

        print("x:", x)
        print("y:", y)
        print("z:", z)
        
        # 绘制图像
        # plt.plot(range(len(lvy)), lvy)
        # plt.plot(range(len(lvx)), lvx)
        # plt.plot(range(len(lvz)), lvz)
        # plt.plot(ly, lx)
        plt.plot(lx, ly)
        plt.title('')
        plt.title('')
        plt.xlabel('x')
        plt.ylabel('')
        plt.grid(True)
        plt.show()
    else:
        print("Please check the IMU data file.")

    
main()
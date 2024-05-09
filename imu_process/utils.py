'''
Author: DMU zhangxianglong
Date: 2024-05-09 10:17:57
* @LastEditTime: 2024-05-09 17:04:01
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/imu_process/utils.py
Description: 
'''
import numpy as np



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
                          [-omga[1][0], omga[0][0],0.0]])
    
    return exp_omga
    
    




    

# import numpy as np

# # 定义旋转矩阵 R_
# R_ = np.array([[1, 0, 0],
#                [0, 1, 0],
#                [0, 0, 1]])

# # 假设 imu.gyro_, bg_ 是 numpy 数组，dt 是一个标量
# imu_gyro = np.array([gx, gy, gz])  # gx, gy, gz 是陀螺仪的三个轴的测量值
# bg = np.array([bx, by, bz])        # bx, by, bz 是陀螺仪的三个轴的偏置

# # 计算旋转矩阵的指数映射
# delta_theta = (imu_gyro - bg) * dt
# exp_delta_theta = np.array([[0, -delta_theta[2], delta_theta[1]],
#                              [delta_theta[2], 0, -delta_theta[0]],
#                              [-delta_theta[1], delta_theta[0], 0]])

# # 计算更新后的旋转矩阵
# R_ = np.dot(R_, np.eye(3) + exp_delta_theta)

# print(R_)

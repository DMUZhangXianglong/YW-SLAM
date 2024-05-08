'''
@Author: DMU zhangxianglong
@Date: 2024-05-08 17:50:54
@LastEditTime: 2024-05-08 17:52:17
@LastEditors: DMU zhangxianglong
@FilePath: \YW-SLAM\imu_process\yw_utils.py
@Description: 
'''


import os

# 读取文件，去除第一个字段不是 "IMU" 的行，并保存到新文件中
def process_file(input_file, output_file):
    with open(input_file, "r") as f_in:
        lines = f_in.readlines()

    filtered_lines = [line for line in lines if line.strip().split()[0] == "IMU"]

    with open(output_file, "w") as f_out:
        f_out.writelines(filtered_lines)

# 获取当前目录
current_directory = os.getcwd()

# 输入文件路径
input_file_path = os.path.join(current_directory, "imu.txt")

# 输出文件路径
output_file_path = os.path.join(current_directory, "imu_out.txt")

# 处理文件
process_file("imu_process/imu.txt", "imu_process/imu_out.txt")
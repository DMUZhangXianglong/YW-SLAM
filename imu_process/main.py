from Imu import IMU
from ImuIntegration import ImuIntegration
import utils
import time
import numpy as np

imu_data_path = "./imu_process/imu_out.txt"
imu_data = np.loadtxt("./imu_process/imu_out.txt")
count_lins = imu_data.shape[0]

def main():
    imu = IMU(imu_data)
    for line in range(count_lins):
        imu.updateData(line)


main()
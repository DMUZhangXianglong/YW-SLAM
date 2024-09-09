'''
* @Author: DMU zhangxianglong
* @Date: 2024-09-09 13:24:58
* @LastEditTime: 2024-09-09 15:47:00
* @LastEditors: DMU zhangxianglong
* @FilePath: /yw_ws/YW-SLAM/ros2_ws/lidar_2d/lidar_2d/icp_2d.py
* @Description: 
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import cv2 
import numpy as np
import sophuspy as sp
import eigenpy


class LaserScanSubscriber(Node):
    # 相当于C++中的构造函数
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/pavo_scan_bottom',  # 你要订阅的LaserScan话题名称
            self.LaserHandler,
            10)
        self.subscription  # 防止未被使用的变量被垃圾回收
        self.image = None

    # 回调函数
    def LaserHandler(self, msg):
        self.Visualize2DScan(msg, self.image)
        # 初始化 eigenpy 的 numpy 兼容模式
        # eigenpy.switchToNumpyArray()

        # 创建 3x3 矩阵
        matrix = eigenpy.Matrix3d()
        matrix.setIdentity()  # 将矩阵初始化为单位矩阵
        print("Matrix:\n", matrix)

        # 创建 3x1 向量
        vector = eigenpy.Vector3d(1.0, 2.0, 3.0)
        print("Vector:\n", vector)
        # self.get_logger().info(f'Received a LaserScan message:{sp.SO2()}')
        # self.get_logger().info(f'Angle Min: {msg.angle_min}')
        # self.get_logger().info(f'Angle Max: {msg.angle_max}')
        # self.get_logger().info(f'Range Data: {msg.ranges[:10]}')  # 打印前10个range数据
        
    def Visualize2DScan(self, msg, image):
        # 检查图像是否为空
        if(image == None):
            image = np.full((800, 800, 3), (255, 255, 255), dtype=np.uint8)
        
        for i in range(len(msg.ranges)):
            if(msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max):
                continue
        
            real_angle = msg.angle_min + i * msg.angle_increment
            x = msg.ranges[i] * np.cos(real_angle)
            y = msg.ranges[i] * np.sin(real_angle)
            
            if (real_angle < msg.angle_min + 30 * np.pi / 180.0 or real_angle > msg.angle_max - 30 * np.pi / 180.0):
                continue
            
            
            


def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

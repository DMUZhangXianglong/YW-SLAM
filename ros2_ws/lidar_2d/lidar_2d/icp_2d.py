'''
* @Author: DMU zhangxianglong
* @Date: 2024-09-09 13:24:58
* @LastEditTime: 2024-09-10 16:16:22
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
        self.color = [0, 255, 0]


    # 回调函数
    def LaserHandler(self, msg):
        self.Visualize2DScan(msg, self.image)
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
            
            psubmap = sp.SE2().inverse() * (sp.SE2() * np.array([x, y]))
            image_x = int(psubmap[0] * 30.0 + 800 / 2)
            image_y = int(psubmap[1] * 30.0 + 800 / 2)
            

            if (image_x >= 0 and image_x < image.shape[0] and image_y >= 0 and image_y < image.shape[1]):
                image[image_y, image_x] = self.color
                cv2.imshow("scan", image)
        
        # 显示画面   
        cv2.waitKey(20) 

# 实现2d icp
class Icp2d():
    # 构造函数
    def __init__(self):
        self.source = None
        self.target = None
    
    def SetSource(self, source):
        self.source = source
    
    def SetTarget(self, target):
        self.target = target
        
    def BuildTargetKdTree(self):
        if(self.target == None):
            return
        
        for i in range(len(self.target)):
            if(self.target.ranges[i] < self.target.range_min or self.target.ranges[i] > self.target.range_max):
                continue
            
            real_angle = self.target.angle_min + i * self.target.angle_increment
            
            p = Point2D
            p.x = self.target.ranges[i] * np.cos(real_angle)
            p.y = self.target.ranges[i] * np.sin(real_angle)
            
            
            
         
            
# 2d 点类
class Point2D:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Point2D(x={self.x}, y={self.y})"


def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

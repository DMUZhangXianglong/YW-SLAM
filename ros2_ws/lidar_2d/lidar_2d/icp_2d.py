'''
* @Author: DMU zhangxianglong
* @Date: 2024-09-09 13:24:58
* @LastEditTime: 2024-09-11 22:11:47
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/ros2_ws/lidar_2d/lidar_2d/icp_2d.py
* @Description: 
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import cv2 
import numpy as np
import sophuspy as sp
from scipy.spatial import KDTree
from tqdm import tqdm


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

        self.last_msg = None
        self.current_msg = None
    
    # 回调函数
    def LaserHandler(self, msg):
        self.current_msg = msg
        if(self.last_msg == None):
            self.last_msg = self.current_msg
            return True

        icp = Icp2d()
        icp.SetSource(self.last_msg)
        icp.SetTarget(self.current_msg)
        pose = sp.SE2()
        icp.AlignGaussNewton(pose)

        # self.Visualize2DScan(msg, self.image)
        # print(len(msg.ranges))

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

        self.target_cloud = None
        self.kdtree = KdTree()
    
    def SetSource(self, source):
        self.source = source
    
    def SetTarget(self, target):
        self.target = target
        self.BuildTargetKdTree()
        
    def BuildTargetKdTree(self):
        if(self.target == None):
            return
        
        # 初始化2d点云
        self.target_cloud = Cloud2d()
    
        for i in range(len(self.target.ranges)):
            if(self.target.ranges[i] < self.target.range_min or self.target.ranges[i] > self.target.range_max):
                continue
            
            real_angle = self.target.angle_min + i * self.target.angle_increment
            
            p = Point2D()
            p.x = self.target.ranges[i] * np.cos(real_angle)
            p.y = self.target.ranges[i] * np.sin(real_angle)
            
            self.target_cloud.push_back(p)
        
        self.target_cloud.width = len(self.target_cloud.points)
        self.kdtree.setInputCloud(self.target_cloud)
            
    def AlignGaussNewton(self, init_pose):
        iterations = 10
        cost = 0
        last_cost = 0
        current_pose = init_pose
        max_dis2 = 0.01
        min_effect_pts = 20

        for iter in range(iterations):
            H = np.zeros((3,3))
            b = np.zeros((3,1))
            cost = 0
            effective_num = 0

            # 遍历source
            for i in range(len(self.source.ranges)):
            # for i in tqdm(len(self.source.ranges)):
                # 距离
                r = self.source.ranges[i]
                
                if (r < self.source.range_min or r > self.source.range_max):
                    continue
                
                angle = self.source.angle_min + i * self.source.angle_increment
                theta = current_pose.so2().log()
                pw = current_pose * np.array([r * np.cos(angle), r * np.sin(angle)])
                pt = Point2D()
                pt.x = pw[0]
                pt.y = pw[1]

                # 最近邻
                nn_idx, dis = self.kdtree.nearestKSearch(pt, 1)
                print(" ")
                print("原始点", pt.x, " ", pt.y)
                print("目标点", self.target_cloud.points[nn_idx], "索引是", nn_idx)
                print("二者距离", dis)
                
                
                
            
            
         
            
# 2d 点类
class Point2D:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Point2D(x={self.x}, y={self.y})"

# 2d 点云类
class Cloud2d:
    def __init__(self):
        # 假设点云是由若干点 (x, y) 组成的
        self.points = []
        self.width = 0

    def push_back(self, point):
        p = np.array([point.x, point.y])
        self.points.append(p)

    def __repr__(self):
        return f"Cloud2d(points={self.points})"
    
# kdtree 类
class KdTree():
    def __init__(self):
        self.kdtree = None

    def setInputCloud(self, cloud):
        if(cloud == None):
            raise ValueError("点云是空的，无法构建 KDTree")
        self.kdtree = KDTree(np.array(cloud.points))

    def nearestKSearch(self, point, k):
        p = np.array([point.x, point.y])
        k_sqr_distances, k_indices = self.kdtree.query(p, k=k)
        return k_indices, k_sqr_distances


# class Scan2d():
#     def __init__(self):
#         self.ranges = None

#     def __repr__(self):
#         return f"Scan2d(ranges={self.ranges})"
        

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

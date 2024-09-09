###
 # @Author: DMU zhangxianglong
 # @Date: 2024-09-09 14:37:17
 # @LastEditTime: 2024-09-09 14:37:18
 # @LastEditors: DMU zhangxianglong
 # @FilePath: /yw_ws/YW-SLAM/ros2_ws/run.sh
 # @Description: 
### 
colcon build
source install/setup.bash
ros2 run lidar_2d icp_2d
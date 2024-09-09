'''
* @Author: DMU zhangxianglong
* @Date: 2024-09-09 13:25:41
* @LastEditTime: 2024-09-09 13:39:58
* @LastEditors: DMU zhangxianglong
* @FilePath: /yw_ws/YW-SLAM/ros2_ws/lidar_2d/setup.py
* @Description: 
'''
from setuptools import find_packages, setup

package_name = 'lidar_2d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='z',
    maintainer_email='347913076@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_2d = lidar_2d.icp_2d:main',
        ],
    },

)

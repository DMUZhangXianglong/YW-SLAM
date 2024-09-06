'''
* @Author: DMU zhangxianglong
* @Date: 2024-09-06 09:16:54
* @LastEditTime: 2024-09-06 17:10:33
* @LastEditors: DMU zhangxianglong
* @FilePath: /YW-SLAM/code/icp.py
* @Description: 使用SVD方法求解 icp 问题; 使用优化方法求解icp问题
'''

import numpy as np
import mayavi.mlab

# 点云文件路径
target = "code/target.txt"
source = "code/source.txt"

# 读取点云数据
def loadData(path):
    data = np.loadtxt(path)
    # 取前 3 列, 表示 X Y Z
    return data[:,:3]

# 显示点云
def clouViewer(cloud):
    x = cloud[:, 0]
    y = cloud[:, 1]
    z = cloud[:, 2]
    vals = 'height'
    if vals == "height":
        col = z
    else:
        col = d
    d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor
    degr = np.degrees(np.arctan(z / d))
    fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(800, 600))
    mayavi.mlab.points3d(x, y, z,
                        col,  # Values used for Color
                        mode="point",
                        colormap='spectral',  # 'bone', 'copper', 'gnuplot'
                        # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                        figure=fig,
                        )
    mayavi.mlab.show()

source_cloud =  loadData(source)
target_cloud =  loadData(target)

# Test #
# print(target_cloud.shape)
# print(source_cloud.shape)
# Test #

# svd 方法求解icp问题
def icp_SVD(pts1, pts2):
    # 转置一下，把每个店组成一个列向量
    pts1 = pts1.T
    pts2 = pts2.T
    
    # 计算质心坐标
    N  = pts1.shape[1]
    p1 = np.sum(pts1, axis=1) / N
    p2 = np.sum(pts2, axis=1) / N
    
   
    # Test #
    # print("p1:", p1)
    # print("p2:", p2)
    # Test #
    
    # 计算去质心坐标
    q1 = pts1 - p1.reshape(3,1)
    q2 = pts2 - p2.reshape(3,1)
    
    # Test #
    # print("q1:", q1)
    # print("q2:", q2)
    # Test #
    
    # 计算 q1*q2^T
    W = np.zeros((3,3))    
    for i in range(N):
        W = W + (np.dot(q1[:,2].reshape(3,1), (q2[:, 2].reshape(3,1)).T))

    # SVD 分解 W
    U, sigma, V = np.linalg.svd(W)
    
    # 求解 R
    R = np.dot(U, V.T)
    if (np.linalg.det(R) < 0):
        R = -R
    
    # 求解 t
    t = p1.reshape(3, 1) - np.dot(R, p2.reshape(3,1))
    
    return R, t


# cloud = np.vstack((source_cloud, target_cloud))
# clouViewer(cloud)
R, t = icp_SVD(source_cloud, target_cloud)
print(R,"\n", t)

# 优化的方法求解 icp 问题

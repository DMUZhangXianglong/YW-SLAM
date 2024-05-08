import numpy as np
import mayavi.mlab
import cam2lidar
import os
from tqdm import tqdm

ExtrinsicMatrix = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                            [6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02],
                            [ 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                            [0.0, 0.0, 0.0, 1.0]])
 
# lidar_path换成自己的.bin文件路径
# T_L = cam2lidar.getLiDARPose("./07poses.txt")
T_C, T_L = cam2lidar.getLiDARPose("./07poses.txt")

files = os.listdir("./07/velodyne/")
cloud_list = []

for n in tqdm(files[:150]):
    pointcloud = np.fromfile("./07/velodyne/"+n, dtype=np.float32, count=-1).reshape([-1, 4])
    cloud_list.append(pointcloud)




# 获取点云原始数据的x y z 组装成 [x y z 1]^T
P = []
k = 0
for cloud in tqdm(cloud_list):
    # print(T_L[k])
    for i in range(cloud.shape[0]):
        p_i = np.zeros((4,1))
        p_i[0][0] = cloud[i][0]
        p_i[1][0] = cloud[i][1]
        p_i[2][0] = cloud[i][2]
        p_i[3][0] = 1.0

        # 雷达到相机
        p_i = np.dot(ExtrinsicMatrix, p_i)
        p_i = np.dot(T_C[k], p_i)

        # 相机到雷达
        # p_i = np.dot(T_L[k], p_i)

        # 取出前行
        p_i = p_i[:3]
        P.append(p_i)
    k = k + 1

P = np.array(P).squeeze(2)

# 取出x y z 
x = P[:, 0]  # x position of point
y = P[:, 1]  # y position of point
z = P[:, 2]  # z position of point


# x = cloud_list[0][:, 0]  # x position of point
# y = cloud_list[0][:, 1]  # y position of point
# z = cloud_list[0][:, 2]  # z position of point

# # Save the extracted coordinates as a .bin point cloud file
# output_filename = "./transformed_point_cloud.bin"
#
# # Stack the x, y, z coordinates horizontally
# transformed_points = np.hstack((x[:, np.newaxis], y[:, np.newaxis], z[:, np.newaxis]))
#
# # Write the points to a binary file
# with open(output_filename, 'wb') as f:
#     transformed_points.tofile(f)



# 显示点云
d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor
degr = np.degrees(np.arctan(z / d))

vals = 'height'
if vals == "height":
    col = z
else:
    col = d

fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(800, 600))
mayavi.mlab.points3d(x, y, z,
                     col,  # Values used for Color
                     mode="point",
                     colormap='spectral',  # 'bone', 'copper', 'gnuplot'
                     # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                     figure=fig,
                     )

mayavi.mlab.show()
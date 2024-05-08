import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

# 打开txt文件
data_poses = []
data_T_Lxyz = []
data_T_L = []
data_T_C = []

ExtrinsicMatrix = np.array([[-1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03],
                            [6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02],
                            [ 9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01],
                            [0.0, 0.0, 0.0, 1.0]])


inverse_ExtrinsicMatrix = np.linalg.inv(ExtrinsicMatrix)
# inverse_ExtrinsicMatrix = ExtrinsicMatrix
# inverse_ExtrinsicMatrix = np.identity(4)



def getLiDARPose(path):
    with open(path, 'r') as file:
        # 逐行读取文件内容
        for line in file:
            # 将每一行按空格分割成元素
            elements = line.strip().split()
            
            # 将元素转换为浮点数
            elements = [float(elem) for elem in elements]
            
            # 将列表转换为NumPy数组
            array = np.array(elements)
            
            # 将一维数组重新形状为3x4的矩阵
            matrix = array.reshape(3, 4)
            
            # 创建一个4x4的零矩阵
            new_matrix = np.zeros((4, 4))
            
            # 将3x4的矩阵复制到新矩阵的前三行
            new_matrix[:3, :] = matrix
            
            # 设置新矩阵的最后一行为[0, 0, 0, 1]
            new_matrix[3, 3] = 1

            data_T_C.append(new_matrix)
            
            # data_poses.append(new_matrix)
            T_L = np.dot(inverse_ExtrinsicMatrix, new_matrix)
            data_T_L.append(T_L)

            # 取出最后一列的前三行
            # T_L_xyz = np.zeros((3,1))
            # T_L_xyz[0][0] = T_L[0][3]
            # T_L_xyz[1][0] = T_L[1][3]
            # T_L_xyz[2][0] = T_L[2][3]
            # data_T_Lxyz.append(T_L_xyz)
            # print(T_L)


    # 绘制散点图  
    # data = list(zip(*data_T_Lxyz))
    # plt.scatter(data[0], data[2])
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.title('cam2lidar')
    # plt.axis('equal')
    # plt.show()
    
    return data_T_C, data_T_L
        


# 读取kitti数据集的真值，并画出
# display ground truth
# import matplotlib.pyplot as plt

# filename = './poses.txt'
# with open(filename, 'r') as fid:
#     lastposition = fid.tell()
#     # print('start position:', lastposition)
#     groundtruth = []
    
#     while True:
#         line = fid.readline()
#         if not line:
#             break
        
#         fid.seek(lastposition)
#         data = fid.readline().split()
#         transform = [float(val) for val in data]
#         transform = [transform[i:i+4] for i in range(0, len(transform), 4)]
        
#         groundtruth.append([transform[0][3], transform[2][3]])
#         print(" ")
#         print(transform[0][3])
#         print(transform[2][3])
#         lastposition = fid.tell()
#         # print('lastposition:', lastposition)

# groundtruth = [[float(val) for val in row] for row in groundtruth]
# groundtruth = list(zip(*groundtruth))
# plt.scatter(groundtruth[0], groundtruth[1])
# plt.axis('equal')
# plt.show()

#################################################################################################
# 仅画xy
import matplotlib.pyplot as plt

# 文件路径
filename = './07poses.txt'  # 请替换为实际的文件路径

# 读取数据
data = []
with open(filename, 'r') as file:
    for line in file:
        values = line.split()
        # 提取每行的第 4、8、12 个数据，并转换为浮点数
        values = [float(values[i]) for i in [3, 7, 11]]
        
        values[0] = values[0]
        data.append(values)

# 绘制散点图
data = list(zip(*data))
plt.scatter(data[0], data[2])
plt.xlabel('x')
plt.ylabel('y')
plt.title('gt')
plt.axis('equal')
plt.show()
#################################################################################################

#################################################################################################
# xyz都画出
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # 文件路径
# filename = './07poses.txt'  # 请替换为实际的文件路径

# # 读取数据
# data = []
# with open(filename, 'r') as file:
#     for line in file:
#         values = line.split()
#         # 提取每行的第 4、8、12 个数据，并转换为浮点数
#         values = [float(values[i]) for i in [3, 7, 11]]
#         data.append(values)

# data = list(zip(*data))

# # 绘制散点图
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# ax.scatter(data[0], data[1], data[2])

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.axis('equal')
# plt.show()
#################################################################################################


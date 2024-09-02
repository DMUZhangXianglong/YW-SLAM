import pandas as pd
import matplotlib.pyplot as plt
from pylab import mpl
mpl.rcParams['font.sans-serif'] = ['Microsoft YaHei']  # 指定默认字体：解决plot不能显示中文问题

def plot_username_frequency(csv_file):
    # 读取CSV文件
    data = pd.read_csv(csv_file)
    
    # 过滤用户名列中内容为“蓝天路体育工作队”的行
    filtered_data = data[data.iloc[:, 9] != '蓝天路体育工作队']
    # filtered_data.iloc[:, 9].replace('步行者', '王天相', inplace=True)
    filtered_data.iloc[:, 9].replace('编译成功', '张向龙', inplace=True)
    # 获取过滤后的用户名列
    username_column = filtered_data.iloc[:, 9]  # 请根据实际情况修改列索引
    
    # 统计每个用户名出现的次数
    username_counts = username_column.value_counts()
    
    # 根据出现次数的大小选择不同的颜色
    colors = plt.cm.coolwarm(username_counts / max(username_counts))  # 使用coolwarm色彩映射
    
    # 绘制垂直条形图
    plt.figure(figsize=(10, 8))
    bars = plt.bar(username_counts.index, username_counts, color=colors, edgecolor='black', alpha=0.7)  # 调整颜色、边缘颜色和透明度
    plt.xlabel('用户名', fontsize=12)
    plt.ylabel('发言次数', fontsize=12)
    plt.title('用户发言次数统计', fontsize=14)
    plt.xticks(fontsize=10, rotation=45)  # 调整刻度字体大小，并旋转刻度标签
    plt.yticks(fontsize=10)
    plt.grid(axis='y', linestyle='--', alpha=0.5)  # 添加网格线
    
    # 在每个柱子上方显示具体数值
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width() / 2, height, str(height), ha='center', va='bottom', fontsize=8)
    
    plt.tight_layout()  # 调整布局，防止文字被裁剪
    plt.show()

# 用法示例
plot_username_frequency("蓝天路体育工作队.csv")


# import pandas as pd
# import matplotlib.pyplot as plt
# from collections import Counter
# from pylab import mpl
# mpl.rcParams['font.sans-serif'] = ['Microsoft YaHei']  # 指定默认字体：解决plot不能显示中文问题

# def plot_username_frequency(csv_file):
#     # 读取CSV文件
#     data = pd.read_csv(csv_file)
    
#     # 过滤用户名列中内容为“啊”的行
#     filtered_data = data[data.iloc[:, 9] != '（新）亚太地区洋务大会']
#     filtered_data.iloc[:, 9].replace('步行者', '王天相', inplace=True)
#     filtered_data.iloc[:, 9].replace('编译成功', '张向龙', inplace=True)
    
#     # 获取过滤后的用户名列
#     username_column = filtered_data.iloc[:, 9]  # 请根据实际情况修改列索引
    
#     # 获取发言内容列
#     content_column = filtered_data.iloc[:, 8]  # 请根据实际情况修改列索引
    
#     # 统计每个用户名发言的频率
#     username_counts = username_column.value_counts()
    
#     # 获取每个用户名发言频率最高的内容
#     most_common_contents = {}
#     for username in username_counts.index:
#         user_content_counts = Counter(content_column[username_column == username])
#         most_common_content = max(user_content_counts, key=user_content_counts.get)
#         most_common_contents[username] = most_common_content
    
#     # 输出结果
#     for username, content in most_common_contents.items():
#         print(f"用户 '{username}' 发言频率最高的内容是: {content}")
    
#     # 根据出现次数的大小选择不同的颜色
#     colors = plt.cm.coolwarm(username_counts / max(username_counts))  # 使用coolwarm色彩映射
    
#     # 绘制垂直条形图
#     plt.figure(figsize=(10, 8))
#     bars = plt.bar(username_counts.index, username_counts, color=colors, edgecolor='black', alpha=0.7)  # 调整颜色、边缘颜色和透明度
#     plt.xlabel('用户名', fontsize=12)
#     plt.ylabel('出现次数', fontsize=12)
#     plt.title('用户名出现次数统计', fontsize=14)
#     plt.xticks(fontsize=10, rotation=45)  # 调整刻度字体大小，并旋转刻度标签
#     plt.yticks(fontsize=10)
#     plt.grid(axis='y', linestyle='--', alpha=0.5)  # 添加网格线
    
#     # 添加文字注释
#     for bar in bars:
#         height = bar.get_height()
#         plt.text(bar.get_x() + bar.get_width() / 2, height, str(height), ha='center', va='bottom', fontsize=8)
    
#     plt.tight_layout()  # 调整布局，防止文字被裁剪
#     plt.show()

# # 用法示例
# plot_username_frequency("file.csv")


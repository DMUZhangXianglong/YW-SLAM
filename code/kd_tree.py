# 二维点的情况
import matplotlib.pyplot as plt

class Node:
    def __init__(self, point, left=None, right=None):
        self.point = point
        self.left = left
        self.right = right

def build_kdtree(points, depth=0):
    if not points:
        return None

    k = len(points[0])  # Dimensionality
    axis = depth % k

    # Sort and choose median as pivot element
    points.sort(key=lambda x: x[axis])
    median = len(points) // 2

    # Recursively build subtrees using the left and right halves of the sorted points
    return Node(
        point=points[median],
        left=build_kdtree(points[:median], depth + 1),
        right=build_kdtree(points[median + 1:], depth + 1)
    )

def plot_kdtree(node, min_x, max_x, min_y, max_y, depth=0):
    if node is not None:
        # Determine the splitting dimension
        k = len(node.point)
        axis = depth % k

        # Plot the point
        plt.plot(node.point[0], node.point[1], 'bo')

        # Plot the splitting line
        if axis == 0:
            plt.plot([node.point[0], node.point[0]], [min_y, max_y], 'k-')
            plot_kdtree(node.left, min_x, node.point[0], min_y, max_y, depth + 1)
            plot_kdtree(node.right, node.point[0], max_x, min_y, max_y, depth + 1)
        else:
            plt.plot([min_x, max_x], [node.point[1], node.point[1]], 'k-')
            plot_kdtree(node.left, min_x, max_x, min_y, node.point[1], depth + 1)
            plot_kdtree(node.right, min_x, max_x, node.point[1], max_y, depth + 1)

# Example usage
points = [(2, 3), (5, 4), (9, 6), (4, 7), (8, 1), (7, 2)]
kdtree = build_kdtree(points)

# Set up the plot
plt.figure(figsize=(8, 8))
plt.axis('equal')
plt.axis('off')

# Plot the KD-tree
plot_kdtree(kdtree, min_x=0, max_x=10, min_y=0, max_y=8)

# Show the plot
plt.show()


# ***********************************************************************

# 三维点的情况
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# class Node:
#     def __init__(self, point, left=None, right=None):
#         self.point = point
#         self.left = left
#         self.right = right

# def build_kdtree(points, depth=0):
#     if not points:
#         return None

#     k = len(points[0])  # Dimensionality
#     axis = depth % k

#     # Sort and choose median as pivot element
#     points.sort(key=lambda x: x[axis])
#     median = len(points) // 2

#     # Recursively build subtrees using the left and right halves of the sorted points
#     return Node(
#         point=points[median],
#         left=build_kdtree(points[:median], depth + 1),
#         right=build_kdtree(points[median + 1:], depth + 1)
#     )


# def plot_kdtree_3d(node, min_x, max_x, min_y, max_y, min_z, max_z, depth=0):
#     if node is not None:
#         # Determine the splitting dimension
#         k = len(node.point)
#         axis = depth % k

#         # Plot the point in 3D space
#         ax.scatter(node.point[0], node.point[1], node.point[2], color='blue')

#         # Plot the splitting plane
#         if axis == 0:
#             ax.plot([node.point[0], node.point[0]], [min_y, max_y], [min_z, min_z], color='black')
#             plot_kdtree_3d(node.left, min_x, node.point[0], min_y, max_y, min_z, max_z, depth + 1)
#             plot_kdtree_3d(node.right, node.point[0], max_x, min_y, max_y, min_z, max_z, depth + 1)
#         elif axis == 1:
#             ax.plot([min_x, max_x], [node.point[1], node.point[1]], [min_z, min_z], color='black')
#             plot_kdtree_3d(node.left, min_x, max_x, min_y, node.point[1], min_z, max_z, depth + 1)
#             plot_kdtree_3d(node.right, min_x, max_x, node.point[1], max_y, min_z, max_z, depth + 1)
#         else:
#             ax.plot([min_x, max_x], [min_y, max_y], [node.point[2], node.point[2]], color='black')
#             plot_kdtree_3d(node.left, min_x, max_x, min_y, max_y, min_z, node.point[2], depth + 1)
#             plot_kdtree_3d(node.right, min_x, max_x, min_y, max_y, node.point[2], max_z, depth + 1)

# # Example usage
# points_3d = [(2, 3, 1), (5, 4, 8), (9, 6, 5), (4, 7, 3), (8, 1, 6), (7, 2, 4), (1, 5, 9), (3, 8, 2)]
# kdtree_3d = build_kdtree(points_3d)

# # Set up the 3D plot
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # Plot the KD-tree in 3D
# plot_kdtree_3d(kdtree_3d, min_x=0, max_x=10, min_y=0, max_y=8, min_z=0, max_z=10)

# # Show the 3D plot
# plt.show()

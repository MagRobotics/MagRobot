import numpy as np
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json

f = open("..\\..\\demo_c\\post_processing\\trajectory_create.txt", "r", encoding="UTF-8")
# f = open("trajectory_create.txt", "r", encoding="UTF-8")

json_str = f.read()
initial_info = json.loads(json_str)
# print(type(initial_info))
lines = initial_info.split('\n')
# print(lines)
coordinates = [eval(line) for line in lines if line]
# print(coordinates)
# 将数据分为 x、y、z
x = [point[0] for point in coordinates]
y = [point[1] for point in coordinates]
z = [point[2] for point in coordinates]
m = [point[3] for point in coordinates]
n = [point[4] for point in coordinates]
p = [point[5] for point in coordinates]


# 打印结果
# print("X values:", x_values)
# print("Y values:", y_values)
# print("Z values:", z_values)

# 示例三维数据点
# t = np.linspace(0, np.pi, 10)
# x = np.cos(t)
# y = np.sin(t)
# z = np.sin(2*t)
# print(type(x))

# 生成三维样条插值的参数
tck_position, u = splprep([x, y, z], s=0)

tck_pose, u_pose = splprep([m, n, p], s=0)


# 在新的参数值上进行插值，生成平滑曲线
new_u = np.linspace(0, 1, 1000)
new_points_position = splev(new_u, tck_position)
new_points_pose = splev(new_u, tck_pose)
# print(new_points_position)
# for i in range(10):
#     distance = np.array([new_points_position[0][i] - new_points_position[0][i+1], new_points_position[1][i] - new_points_position[1][i+1], new_points_position[2][i] - new_points_position[2][i+1]])
#     print(np.linalg.norm(distance))


output = new_points_position + new_points_pose

converted_list = [arr.tolist() for arr in output]
json_str = json.dumps(converted_list, ensure_ascii=False)
# f = open('..\\..\\demo_c\\post_processing\\trajectory_output.txt', "w", encoding="UTF-8")
f = open('trajectory_output.txt', "w", encoding="UTF-8")
f.truncate(0)
f.write(json_str)
f.close()

# print(len(output))
'''
只需将new_points[0][1][2]分别存到三个文件中即可，以便后续控制运用
'''
# print(new_points[0])
# 绘制原始数据点和插值结果
fig, (ax, bx) = plt.subplots(1, 2, figsize=(12, 6), subplot_kw={'projection': '3d'})
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c='r', marker='o', label='Original Data')
ax.plot(*new_points_position, label='Smooth Curve')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.legend()

# bx = fig.add_subplot(111, projection='3d')
bx.scatter(m, n, p, c='r', marker='o', label='Original Data')
bx.plot(*new_points_pose, label='Smooth Curve')
bx.set_xlabel('M')
bx.set_ylabel('N')
bx.set_zlabel('P')
plt.legend()

plt.savefig('..\\..\\demo_c\\post_processing\\plot_trajectory.png')
# plt.savefig('plot_trajectory.png')


plt.show()



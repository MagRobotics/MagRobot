import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json

f = open("..\\..\\demo_c\\post_processing\\cunchu_x.txt", "r", encoding="UTF-8")
# f = open("cunchu_x.txt", "r", encoding="UTF-8")
j_position_x = f.read()
position_x = json.loads(j_position_x)
f = open("..\\..\\demo_c\\post_processing\\cunchu_y.txt", "r", encoding="UTF-8")
# f = open("cunchu_y.txt", "r", encoding="UTF-8")
j_position_y = f.read()
position_y = json.loads(j_position_y)
f = open("..\\..\\demo_c\\post_processing\\cunchu_z.txt", "r", encoding="UTF-8")
# f = open("cunchu_z.txt", "r", encoding="UTF-8")
j_position_z = f.read()
position_z = json.loads(j_position_z)

f = open("..\\..\\demo_c\\post_processing\\estimate_x.txt", "r", encoding="UTF-8")
# f = open("estimate_x.txt", "r", encoding="UTF-8")
j_estimate_x = f.read()
estimate_x = json.loads(j_estimate_x)
f = open("..\\..\\demo_c\\post_processing\\estimate_y.txt", "r", encoding="UTF-8")
# f = open("estimate_y.txt", "r", encoding="UTF-8")
j_estimate_y = f.read()
estimate_y = json.loads(j_estimate_y)
f = open("..\\..\\demo_c\\post_processing\\estimate_z.txt", "r", encoding="UTF-8")
# f = open("estimate_z.txt", "r", encoding="UTF-8")
j_estimate_z = f.read()
estimate_z = json.loads(j_estimate_z)


# 绘制三维图形
fig = plt.figure(figsize=(6.4, 4.8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(position_x, position_y, position_z, label='Line 1', color='b')
ax.plot(estimate_x, estimate_y, estimate_z, label='Line 2', color='r')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# ax.set_ylim(-0.01, 0.01)
# ax.set_zlim(1.05, 1.15)
plt.title('3D Spiral')

plt.savefig('..\\..\\demo_c\\post_processing\\plot_pos.png')
plt.show()



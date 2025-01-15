import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json

# f = open("cunchu_x.txt", "r", encoding="UTF-8")
f = open("..\\..\\demo_c\\post_processing\\cunchu_x.txt", "r", encoding="UTF-8")
j_position_x = f.read()
position_x = json.loads(j_position_x)
# f = open("cunchu_y.txt", "r", encoding="UTF-8")
f = open("..\\..\\demo_c\\post_processing\\cunchu_y.txt", "r", encoding="UTF-8")
j_position_y = f.read()
position_y = json.loads(j_position_y)
# f = open("cunchu_z.txt", "r", encoding="UTF-8")
f = open("..\\..\\demo_c\\post_processing\\cunchu_z.txt", "r", encoding="UTF-8")
j_position_z = f.read()
position_z = json.loads(j_position_z)

# f = open("plan_x.txt", "r", encoding="UTF-8")
f = open("..\\..\\demo_c\\post_processing\\plan_x.txt", "r", encoding="UTF-8")
j_plan_x = f.read()
plan_x = json.loads(j_plan_x)
# f = open("plan_y.txt", "r", encoding="UTF-8")
f = open("..\\..\\demo_c\\post_processing\\plan_y.txt", "r", encoding="UTF-8")
j_plan_y = f.read()
plan_y = json.loads(j_plan_y)
# f = open("plan_z.txt", "r", encoding="UTF-8")
f = open("..\\..\\demo_c\\post_processing\\plan_z.txt", "r", encoding="UTF-8")
j_plan_z = f.read()
plan_z = json.loads(j_plan_z)


# 绘制三维图形
fig = plt.figure(figsize=(6.4, 4.8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(position_x, position_y, position_z, label='Line 1', color='b')
ax.plot(plan_x, plan_y, plan_z, label='Line 2', color='r')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# ax.set_ylim(-6, 4)
plt.title('3D Spiral')

plt.savefig('..\\..\\demo_c\\post_processing\\plot_con.png')
plt.show()



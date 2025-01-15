import numpy as np
from numpy import *
import math
import json


class collison:
    def __init__(self,obstacle_list):
        self.obstacle_list = obstacle_list
    def check_qiu_collision(self, x1,y1,z1,x2,y2,z2):
        for(ox,oy,oz,size)in self.obstacle_list:
            a = np.asarray([x1,y1,z1])
            b = np.asarray([x2,y2,z2])
            p = np.asarray([ox,oy,oz])
            ab = b - a
            ap = p - a
            bp = p - b
            r = np.dot(ap, ab) / (np.linalg.norm(ab)) ** 2
            c = ab * r + a
            cp = p - c
            # 计算投影长度，并做正则化处理

            # 分了三种情况
            if r > 0 and r < 1:
                dis = np.linalg.norm(cp)
            elif r >= 1:
                dis = np.linalg.norm(bp)
            else:
                dis = np.linalg.norm(ap)
            if dis < size:
                avoid_list.append(a)
                avoid_list.append(b)
                avoid_list.append(p)
                avoid_list.append(size)
                return False
        return True
    def Avoid_obstacle(self,a,b,p,size):
        ab = b - a
        ap = p - a
        bp = p - b
        r = np.dot(ap, ab) / (np.linalg.norm(ab)) ** 2
        c = ab*r +a
        cp =p - c
        d = np.linalg.norm(cp)
        if(d==0):
            if(ab[0]!=0):
                nx = -(ab[1]+ab[2])/ab[0]
                ny = 1
                nz = 1
                n = np.asarray([nx,ny,nz])
                print(ab[1])
            elif(ab[1]!=0):
                nx = -1
                ny = (ab[0]+ab[2])/ab[1]
                nz = -1
                print(ab[1])
                n = np.asarray([nx, ny, nz])
            else:
                nx = -1
                ny = -1
                nz = (ab[0] + ab[1]) / ab[2]
                print(ab[1])
                n = np.asarray([nx, ny, nz])
            n0 = n/np.linalg.norm(n)
            an = a + n0*size
            bn = b + n0*size
            return an,bn
        else:
            direction = cp / d
            anew = a - direction*(size-d)
            bnew = b - direction*(size-d)*1.2
            return anew,bnew





points = []
avoid_list = []
avoid_points = []
f = open("..\\..\\demo_c\\post_processing\\centerline.txt", "r", encoding="UTF-8")
# f = open("centerline.txt", "r", encoding="UTF-8")
json_str = f.read()
pointslie = json.loads(json_str)
for i in range(0,len(pointslie[0])):
    pointshang = [pointslie[0][i],pointslie[1][i],pointslie[2][i]]
    points.append(pointshang)

print(points)
f = open("..\\..\\demo_c\\GUI_interface\\ceshi.txt", "r", encoding="UTF-8")
# f = open("ceshi.txt", "r", encoding="UTF-8")
json_str = f.read()
info = json.loads(json_str)
My_object = collison(info[6][0]["barrier_points"])
for i in range(1,len(points)):
    x1 = points[i-1][0]
    y1 = points[i-1][1]
    z1 = points[i-1][2]
    x2 = points[i][0]
    y2 = points[i][1]
    z2 = points[i][2]
    result = My_object.check_qiu_collision(x1,y1,z1,x2,y2,z2)
    if(result == False):

        points[i - 1] , points[i] = My_object.Avoid_obstacle(avoid_list[0],avoid_list[1],avoid_list[2],avoid_list[3])
        change = [i-1,i,points[i-1][0],points[i-1][1],points[i-1][2],points[i][0],points[i][1],points[i][2],x1,y1,z1,x2,y2,z2]
        avoid_points.append(change)
        avoid_list.clear()
    else:
        pass
for l in range(1,len(points)):
    x11 = points[l - 1][0]
    y11 = points[l - 1][1]
    z11 = points[l - 1][2]
    x21 = points[l][0]
    y21 = points[l][1]
    z21 = points[l][2]
    result = My_object.check_qiu_collision(x11, y11, z11, x21, y21, z21)
    if(result == False):
        print("完蛋",l)

pointsx=[]
pointsy=[]
pointsz=[]
for i in range(0,len(points)):
    pointsx.append(points[i][0])
    pointsy.append(points[i][1])
    pointsz.append(points[i][2])

pointsout = []
pointsout.append(pointsx)
pointsout.append(pointsy)
pointsout.append(pointsz)

with open("..\\..\\demo_c\\post_processing\\centerline_avoid.txt", "w") as file:
# with open("centerline_avoid.txt", "w") as file:
    file.write(str(pointsout))

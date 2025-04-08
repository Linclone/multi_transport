#!/usr/bin/env python

import rospy
import rospkg

import math
import random

from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import util
import configparser, os


class Base_confi():
    """读取../config/config.ini配置文件"""
 
    def __init__(self):
        self.configfile_path = os.path.join(os.path.dirname(__file__), '..', 'config/config.ini')
        self.conf = configparser.ConfigParser()
        self.conf.read(self.configfile_path)
 
    """获取指定节点下指定option的值，返回为字符串类型"""
 
    def config_get(self, title, value):
        return int(self.conf.get(title, value))
 
    """返回conf.ini文件路径"""
 
    def config_path(self):
        return self.configfile_path
 
    """修改conf.ini文件数据，如果section不存在的话，先进行判断然后进行添加再修改"""
 
    def config_modify(self, section, option, value):
        if not self.conf.has_section(section):
            self.conf.add_section(section)
        self.conf.set(section, option, value)
        with open(self.configfile_path, "w") as f:
            self.conf.write(f)

def norm(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    
# 判断p1到p2的连线上是否有障碍物
def check_obs(map: np.array, p1, p2) -> bool:
    sz=map.shape
    d=norm(p1,p2)
    ling_angle=math.atan2(p2[1]-p1[1],p2[0]-p1[0])
    for r in range(0,math.floor(d+1)):
        x = math.floor(p1[0] + r*math.cos(ling_angle))
        y = math.floor(p1[1] + r*math.sin(ling_angle))

        if (x>0) & (x<sz[1]) & (y>0) & (y<sz[0]):
            if map[y,x] == 0:
                return 0
        else:
            return 0
    
    return 1

# 判断p1点处刚体可能存在的角度
def rigid_exist(map: np.array, p1, rigid_size, scan_rate):
    theta=np.array([])
    exist=0
    sz=map.shape
    x=p1[0]
    y=p1[1]
    a=rigid_size[0]
    b=rigid_size[1]
    fi_range=np.linspace(-1.57,1.57,scan_rate)
    for fi_index in range(0,fi_range.shape[0]):
        fi=fi_range[fi_index]
        exist_fi=1
        if (x-a/2*math.cos(fi)<0) | (x+a/2*math.cos(fi)>sz[0]) | (y-a/2*math.sin(fi)<0) | (y+a/2*math.sin(fi)>sz[1]) :
            continue
        else:
            space1x=np.linspace(x-a/2*math.cos(fi),x+a/2*math.cos(fi),a)
            space1y=np.linspace(y-a/2*math.sin(fi),y+a/2*math.sin(fi),a)
            for i in range(0,a):
                xx=int(round(space1x[i]))
                yy=int(round(space1y[i]))
                if (xx-b/2*math.sin(fi)<0) | (xx+b/2*math.sin(fi)>sz[0]) | (yy+b/2*math.cos(fi)<0) | (yy-b/2*math.cos(fi)>sz[1]) :
                    exist_fi=0
                else:
                    space2x=np.linspace(xx-b/2*math.sin(fi),xx+b/2*math.sin(fi),b)
                    space2y=np.linspace(yy+b/2*math.cos(fi),yy-b/2*math.cos(fi),b)
                    for ii in range(0,b):
                        x_cur=int(round(space2x[ii]))
                        y_cur=int(round(space2y[ii]))
                        if(map[y_cur,x_cur]==0):
                            exist_fi=0
                            break
                        # if abs(fi-1.0715873)<0.005:
                        #     print(fi)
                        #     map[y_cur,x_cur]=0
                if exist_fi==0:
                    break
        if exist_fi:
            theta=np.append(theta,fi)
            exist=1
    return [exist,theta]



#  参数
#  map：空间地图，为图片矩阵，障碍物取值为0
#  S：起始点坐标
#  E：终点坐标
#  step:步长，由需求和经验自行设定
#  pro:（0-1）按一定的概率以终点作为P_rand，能够加快路径搜索速度
#  R：潜在父节点的搜索半径
#  (a,b)矩形长宽
def RRT_star(map: np.array, S: np.array, E: np.array, step, pro, R, rigid_size,scan_rate):
    path=np.array([E])
    thetas=np.array([])
    sz=map.shape

    Tree_list = np.array([S])
    tr_theta=np.array([0])

    cost_list = np.array([0])
    f_index_list = np.array([0])
    p_new = np.array(S)
    distance = norm(p_new,E)
    DISTANCE=distance
    outflags=[1,1,1,1,1,1,1,1,1]
    distance_last = distance

    while distance > step:
        # print(distance,Tree_list,'\n')
        # if distance != distance_last:
        #     print(distance)
        #     distance_last=distance
        if outflags[0] & (distance < DISTANCE*0.9):
            print("generated 10%")
            outflags[0]=0
        elif outflags[1] & (distance < DISTANCE*0.8):
            print("generated 20%")
            outflags[1]=0
        elif outflags[2] & (distance < DISTANCE*0.7):
            print("generated 30%")
            outflags[2]=0
        elif outflags[3] & (distance < DISTANCE*0.6):
            print("generated 40%")
            outflags[3]=0
        elif outflags[4] & (distance < DISTANCE*0.5):
            print("generated 50%")
            outflags[4]=0
        elif outflags[5] & (distance < DISTANCE*0.4):
            print("generated 60%")
            outflags[5]=0
        elif outflags[6] & (distance < DISTANCE*0.3):
            print("generated 70%")
            outflags[6]=0
        elif outflags[7] & (distance < DISTANCE*0.2):
            print("generated 80%")
            outflags[7]=0
        elif outflags[8] & (distance < DISTANCE*0.1):
            print("generated 90%")
            outflags[8]=0



        if random.random() < pro:
            P_rand=E
        else:
            P_rand = np.array([random.random()*sz[1],random.random()*sz[0]])

        mindiff=9999.9
        index=-1
        for i in range(0,len(Tree_list)):
            diff=norm(P_rand,Tree_list[i])
            if diff<mindiff:
                mindiff=diff
                index=i
        
        P_near = Tree_list[index]
        line_angle=math.atan2(P_rand[1]-P_near[1],P_rand[0]-P_near[0])
        x=P_near[0] + step*math.cos(line_angle)
        y=P_near[1] + step*math.sin(line_angle)
        p_new=np.array([x,y])
        
        [exi,theta]=rigid_exist(map,p_new,rigid_size,scan_rate)
        if exi==0:
            continue

        Tree_len = len(Tree_list)
        path_cost = cost_list[index] + step
        f_index = index

        for i in range(0,Tree_len):
            dist = norm(p_new,Tree_list[i])

            if dist <= R:
                if (cost_list[i] + norm(p_new,Tree_list[i])) < path_cost:
                    if check_obs(map,P_near,p_new):
                        P_near=Tree_list[i]
                        f_index=i

        if check_obs(map,P_near,p_new):
            f_index_list=np.append(f_index_list,f_index)

            Tree_list=np.append(Tree_list,[p_new],axis=0)
            tr_theta=np.append(tr_theta,theta[math.floor(theta.size/3)]) # 角度选取问题待解决
            cost_new=cost_list[f_index] + norm(p_new, P_near)
            cost_list=np.append(cost_list,cost_new)

            # 重新布线
            distance = norm(p_new,E)

            Tree_len=Tree_list.shape[0]
            path_cost=cost_list[index] + step
            f_index = index
            for i in range(0,Tree_len):
                dist = norm(p_new , Tree_list[i])

                if dist <= R:
                    if (cost_new + norm(p_new,Tree_list[i])) < cost_list[i]:
                        if check_obs(map,p_new,Tree_list[i]):
                            condition= Tree_list==Tree_list[i]
                            condition= condition[:,0] & condition[:,1]
                            node_index=np.where(condition)[0]
                            f_index=f_index_list[node_index]

                            f_index_list[i]=Tree_len-1
                            cost_list[i]=cost_new + norm(p_new, Tree_list[i])

        else:
            continue
        
    E_f_index=len(Tree_list)-1
    Tree_list = np.append(Tree_list,[E],axis=0)
    f_index_list=np.append(f_index_list,E_f_index)

    # 搜索路径：
    f_index = f_index_list[-1]
    while f_index != 0:
        father_node = Tree_list[f_index]
        path = np.append([father_node],path,axis=0)
        thetas = np.append(tr_theta[f_index],thetas)
        current_node = father_node
        condition= Tree_list==current_node
        condition= condition[:,0] & condition[:,1]
        current_index=np.where(condition)[0][0]
        f_index = f_index_list[current_index]

    path = np.append([S],path,axis=0)
    thetas = np.append(0,thetas)
    
    print("RRT finish!\n")
    return path



def judge_theta(theta,scan_rate2):
    flag=1
    thetas=np.append(theta,theta+3.14)
    out=np.array([])
    last=thetas[0]
    temp=thetas[0]
    times=1
    for i in range(1,len(thetas)):
        if abs(thetas[i]-last) > 3*3.14/scan_rate2:
            if temp/times <= 1.57:
                out=np.append(out,temp/times)
            temp=thetas[i]
            last=thetas[i]
            times=1
            flag=0
        else:
            temp+=thetas[i]
            last=thetas[i]
            times+=1
    if temp/times <= 1.57:
        out=np.append(out,temp/times)
    return out,flag


def gazebo2pgm(pgmsize, realmap_size, P):
    # 仅正方形
    return [pgmsize*0.5+P[0]*pgmsize/realmap_size, pgmsize*0.5-P[1]*pgmsize/realmap_size]

def pgm2gazebo(pgmsize, realmap_size, Q):
    return [-int(realmap_size/pgmsize*(pgmsize*0.5-Q[0])),int(realmap_size/pgmsize*(pgmsize*0.5-Q[1]))]

def quat2eu(z,w):
    return math.atan2(2 * w*z , 1 - 2 * z*z)
        


if __name__ == '__main__':

    rospy.init_node("global_planning")

    rospack = rospkg.RosPack()
 
    # 获取名为'your_package_name'的包的路径
    package_path = rospack.get_path('global_planning')
    path=package_path + '/map/world.pgm'
    confs = Base_confi()
    
    print('\n-----using map in', path)
    map=util.read_pgm(path)
    print(map.shape)

    P=[9,-8]
    Q=gazebo2pgm(600,30,P)

    print(map[int(Q[1]),int(Q[0])])

    theta = 0.8

    for j in range(-8,9):
        map[int(round(Q[1]+j*math.sin(theta))),int(round(Q[0]+j*math.cos(theta)))]=0

    x1 = Q[0]+8*math.cos(theta)
    y1 = Q[1]+8*math.sin(theta)

    x2 = Q[0]-8*math.cos(theta)
    y2 = Q[1]-8*math.sin(theta)
    print(x1,y1)
    print(x2,y2)
    atantheta = math.atan2(y1-y2,x1-x2)
    atantheta2 = math.atan2(y2-y1,x2-x1)
    print(atantheta)
    print(atantheta2)

    plt.imshow(map,cmap="gray", vmin=0,vmax=1)
    plt.show()

    print(pgm2gazebo(600,30,Q))



    
    # # 设置矩形大小
    # rigid_size=np.array([confs.config_get('RIGID','a'),confs.config_get('RIGID','b')])

    # # 设置起点终点
    # S=np.array([confs.config_get('RIGID','Sx'),confs.config_get('RIGID','Sy')])
    # E=np.array([confs.config_get('PLANNING','Ex'),confs.config_get('PLANNING','Ey')])

    # scan_rate = confs.config_get('PLANNING','scan_rate')
    # scan_rate2 = confs.config_get('PLANNING','scan_rate2')

    # # print(check_obs(map,[168,250],[178,248]))
    # # map[250,168]=0
    # # map[248,178]=0
    # # map[S[1],S[0]]=0
    # # map[E[1],E[0]]=0
    # # plt.imshow(map,cmap="gray", vmin=0,vmax=1)
    # # plt.show()

    # #  参数
    # #  map：空间地图，为图片矩阵，障碍物取值为0
    # #  S：起始点坐标
    # #  E：终点坐标
    # #  step:步长，由需求和经验自行设定
    # #  pro:（0-1）按一定的概率以终点作为P_rand，能够加快路径搜索速度
    # #  R：潜在父节点的搜索半径
    # #  (a,b)矩形长宽
    # #  scan_rate 矩形位姿偏角扫描精度 = pi / scan_rate



    # [flag1,theta]=rigid_exist(map,S,rigid_size,scan_rate)
    # [flag2,theta]=rigid_exist(map,E,rigid_size,scan_rate)
    # if flag1 & flag2:
    #     costmin=9999
    #     for j in range(0,5):
    #         path0t = RRT_star(map,S,E,70,0.7,30,rigid_size,scan_rate)
    #         cost_t=0.0
    #         for i in range(1,len(path0t)):
    #             cost_t += norm(path0t[i-1],path0t[i])
    #         if cost_t < costmin:
    #             path0 = path0t

    

    #     path=np.array([S])
    #     for i in range(1,len(path0)):
    #         patht=RRT_star(map,path0[i-1],path0[i],10,0.7,15,rigid_size,scan_rate)
    #         path=np.append(path,patht[1:],axis=0)
            
        

    #     last_angle = confs.config_get('RIGID','angle0')
    #     angles=np.array([])
    #     for i in range(0,len(path)):
    #         [flag,theta]=rigid_exist(map,path[i],rigid_size,scan_rate2)
    #         # print(path[i],'  ============  ',theta)
    #         outtemp,flag=judge_theta(theta,scan_rate2)
    #         angtemp=-20
    #         d=100
    #         if len(outtemp)==0:
    #             print('fault in scanning -- G1 ')

    #         if flag:
    #             angtemp=last_angle
    #         else:
    #             for j in range(0,len(outtemp)):
    #                 if abs(outtemp[j]-last_angle) < d:
    #                     angtemp=outtemp[j]
    #                     d=angtemp-last_angle
    #         angles=np.append(angles,angtemp)
    #         last_angle=angtemp



            
    #     for i in range(0,len(path)):
    #         for j in range(-30,30):
    #             map[int(round(path[i][1]+j*math.sin(angles[i]))),int(round(path[i][0]+j*math.cos(angles[i])))]=0
    #         for j in range(-5,6):
    #             map[int(round(path[i][1]))+j,int(round(path[i][0]))-j]=0


        # plt.imshow(map,cmap="gray", vmin=0,vmax=1)
        # plt.show()



    # else:
    #     print("起点/终点设置错误！-- G0 ")



    # P=np.array([319 ,229])
    # [flag1,theta]=rigid_exist(map,P,rigid_size,64)
    # print(theta)
    # map[229,319]=0
    # angles=1.0715873
    # for j in range(-30,30):
    #     map[int(round(P[1]+j*math.sin(angles))),int(round(P[0]+j*math.cos(angles)))]=0

    # outtemp,flag=judge_theta(theta,64)
    # print(outtemp)
    # plt.imshow(map,cmap="gray", vmin=0,vmax=1)
    # plt.show()
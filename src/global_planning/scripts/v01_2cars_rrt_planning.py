#!/usr/bin/env python

# The last version for dual-robot transportation

import rospy
import rospkg

import math
import random

from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import configparser, os
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

import util

random.seed(2025) # for testing

class husky_ctrl:
    def __init__(self, robot_name):
        self.begin = 1
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.real_pose = Pose()
        self.real_twist = Twist()
        self.cmdtwist_pub = rospy.Publisher(self.robot_name+'/cmd_vel', Twist, queue_size=5)
        print("send command to ", self.robot_name+'/cmd_vel')
        self.pose_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.state_callback)
        print("receive from /gazebo/model_states")
        while(self.begin):
            pass

    def state_callback(self, data):
        for i,x in enumerate(data.name):
            if x == self.robot_name:
                self.real_pose = data.pose[i]
                self.real_twist = data.twist[i]
        self.begin = 0

    def husky_cmd(self, x, z):
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
        self.cmdtwist_pub.publish(self.cmd_twist)

    def pose_get(self):
        return self.real_pose

    def twist_get(self):
        return self.real_twist


class PurePursuit:
    def __init__(self, path):
        self.path = path
        self.lookahead_distance = 0.4  # 预瞄距离
        self.current_idx = 0

    def search_target(self, pose):
        # 寻找最近的路径点
        dx = [pose.position.x - x for x, y in self.path]
        dy = [pose.position.y - y for x, y in self.path]
        d = np.hypot(dx, dy)
        nearest_idx = np.argmin(d)

        # 寻找预瞄点
        for i in range(nearest_idx, len(self.path)):
            if np.hypot(self.path[i][0]-pose.position.x, self.path[i][1]-pose.position.y) >= self.lookahead_distance:
                self.current_idx = i
                return self.path[i]
        return self.path[-1]



class Base_confi():
    """读取../config/config.ini配置文件"""
 
    def __init__(self):
        self.configfile_path = os.path.join(os.path.dirname(__file__), '..', 'config/config.ini')
        self.conf = configparser.ConfigParser()
        self.conf.read(self.configfile_path)
 
    """获取指定节点下指定option的值"""
 
    def config_get(self, title, value):
        return str(self.conf.get(title, value))
 
    """返回config.ini文件路径"""
 
    def config_path(self):
        return self.configfile_path
 
    """修改config.ini文件数据，如果section不存在的话，先进行判断然后进行添加再修改"""
 
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
    return np.array([pgmsize*0.5+P[0]*pgmsize/realmap_size, pgmsize*0.5-P[1]*pgmsize/realmap_size])

def pgm2gazebo(pgmsize, realmap_size, Q):
    return np.array([-int(realmap_size/pgmsize*(pgmsize*0.5-Q[0])),int(realmap_size/pgmsize*(pgmsize*0.5-Q[1]))])
        
def quat2eu(z,w):
    # -3.14 ~ 3.14 , 0 指向pgm图像左侧，上半为正
    eu = math.atan2(2 * w*z , 1 - 2 * z*z)
    return eu

def mapeu_2_worldeu(eu):
    if eu>=0:
        out = math.pi - eu
    else:
        out = - math.pi - eu
    return out


def worldeu_2_mapeu(eu):
    if eu>=0:
        out = math.pi - eu
    else:
        out = - math.pi - eu
    return out

def path_gen(cur, dest, theta1, theta2, sampling_rate, L):
    tau = np.linspace(0, 1, sampling_rate)
    dtheta=theta2-theta1
    if dtheta > np.pi:
        dtheta -= 2 * np.pi
    elif dtheta < -np.pi:
        dtheta += 2 * np.pi


    x1 = cur[0] + (dest[0] - cur[0]) * tau + L/2 * np.cos(theta1 + (dtheta) * tau)
    y1 = cur[1] + (dest[1] - cur[1]) * tau + L/2 * np.sin(theta1 + (dtheta) * tau)

    x2 = cur[0] + (dest[0] - cur[0]) * tau - L/2 * np.cos(theta1 + (dtheta) * tau)
    y2 = cur[1] + (dest[1] - cur[1]) * tau - L/2 * np.sin(theta1 + (dtheta) * tau)

    return list(zip(x1, y1)), list(zip(x2, y2))
    
def pure_pursuit_control(pose, target, V):
    alpha = np.arctan2(target[1] - pose.position.y, target[0] - pose.position.x) - quat2eu(pose.orientation.z, pose.orientation.w)
    omega = 2 * V * np.sin(alpha) / np.hypot(target[0]-pose.position.x, target[1]-pose.position.y)
    return omega



if __name__ == '__main__':

    rospy.init_node("global_planning")

    rospack = rospkg.RosPack()

    # 获取当前包的路径
    package_path = rospack.get_path('global_planning')
    path=package_path + '/map/world.pgm'
    confs = Base_confi()


    
    map=util.read_pgm(path)
    print('\n-----using map in', path, ', mapshape = ', map.shape)
    
    # 设置矩形大小
    rigid_size=np.array([float(confs.config_get('RIGID','a')),float(confs.config_get('RIGID','b'))])
    rigid_size=(rigid_size*600/30).astype(int)

    # 设置起点终点
    S=np.array(gazebo2pgm(600,30,[int(confs.config_get('RIGID','Sx')),int(confs.config_get('RIGID','Sy'))]))
    E=np.array(gazebo2pgm(600,30,[int(confs.config_get('PLANNING','Ex')),int(confs.config_get('PLANNING','Ey'))]))
    print('起点：', S)
    print('终点：', E)

    scan_rate = int(confs.config_get('PLANNING','scan_rate'))
    scan_rate2 = int(confs.config_get('PLANNING','scan_rate2'))

    init_car1pos = np.array([float(confs.config_get('PLANNING','car1_x')),float(confs.config_get('PLANNING','car1_y'))])
    init_car2pos = np.array([float(confs.config_get('PLANNING','car2_x')),float(confs.config_get('PLANNING','car2_y'))])

    l_2 = 0.5*norm( init_car1pos, init_car2pos ) # 小车距中心的像素距离，即两小车距离的一半

    [flag1,theta]=rigid_exist(map,S,rigid_size,scan_rate)
    [flag2,theta]=rigid_exist(map,E,rigid_size,scan_rate)
    if flag1 & flag2:
        while True:
            print("RRT init...")
            costmin=9999
            for j in range(0,5):
                path0t = RRT_star(map,S,E,70,0.7,30,rigid_size,scan_rate)
                print('RRT_rough Finished! -- ',j+1,'/5')
                cost_t=0.0
                for i in range(1,len(path0t)):
                    cost_t += norm(path0t[i-1],path0t[i])
                if cost_t < costmin:
                    path0 = path0t

        

            path=np.array([S])
            for i in range(1,len(path0)):
                patht=RRT_star(map,path0[i-1],path0[i],30,0.7,15,rigid_size,scan_rate)
                print('RRT_detailed Finished! -- ',i,'/',len(path0)-1)
                path=np.append(path,patht[1:],axis=0)
                
            

            print('angle generating...')
            last_angle = float(confs.config_get('RIGID','angle0'))
            angles=np.array([last_angle])
            for i in range(1,len(path)):
                [flag,theta]=rigid_exist(map,path[i],rigid_size,scan_rate2)
                outtemp,flag=judge_theta(theta,scan_rate2)
                angtemp=-20
                d=100
                if len(outtemp)==0:
                    print('fault in scanning -- G1 ')
                    exit(-1)

                if flag:
                    angtemp=last_angle
                else:
                    for j in range(0,len(outtemp)):
                        if abs(outtemp[j]-last_angle) < d:
                            angtemp=outtemp[j]
                            d=angtemp-last_angle
                angles=np.append(angles,angtemp)
                last_angle=angtemp

            last_angle = angles[0]
            for i in range(1,len(angles)):
                if abs(angles[i]-last_angle)>math.pi / 2:
                    if angles[i] <= 0:
                        angles[i] = angles[i] + math.pi
                    else:
                        angles[i] = angles[i] - math.pi
                last_angle = angles[i]

            print('angles=' ,angles)           

            for i in range(0,len(path)):
                for j in range(-int(20*l_2),int(20*l_2)+1):
                    map[int(round(path[i][1]+j*math.sin(angles[i]))),int(round(path[i][0]+j*math.cos(angles[i])))]=0
                for j in range(-5,5+1):
                    map[int(round(path[i][1]))+j,int(round(path[i][0]))-j]=0


            plt.imshow(map,cmap="gray", vmin=0,vmax=1)
            plt.show()

            # break_input = input("===> accept? [y/n]: ")
            # if break_input == 'y':
            #     break
            break


        car1 = husky_ctrl('husky_1')
        car2 = husky_ctrl('husky_2')


        vel_max = float(confs.config_get('PLANNING','Vm'))
        yaw_max = float(confs.config_get('PLANNING','Yawm'))
        pose1 = car1.pose_get()
        pose2 = car2.pose_get()

        
        duration = 0.02
        
        last_pose = S
        last_theta = float(confs.config_get('RIGID','angle0'))


        for gostep in range(1,len(path)):

            print('controling husky... ----', gostep, '/', len(path)-1)

            destination = pgm2gazebo(600, 30, path[gostep].astype(int))
            dest_angle = mapeu_2_worldeu(angles[gostep])
            
            pose1 = car1.pose_get()
            pose2 = car2.pose_get()
            cur_car1 = np.array([pose1.position.x, pose1.position.y])
            cur_car2 = np.array([pose2.position.x, pose2.position.y])

            cur_pos = (cur_car1+cur_car2)/2
            cur_theta = math.atan2((cur_car1[1]-cur_car2[1]), (cur_car1[0]-cur_car2[0]))      # -pi 到 pi      
            print(dest_angle,'----',cur_theta)



            fi1_cur = worldeu_2_mapeu(quat2eu(pose1.orientation.z, pose1.orientation.w))
            fi2_cur = worldeu_2_mapeu(quat2eu(pose2.orientation.z, pose2.orientation.w))

            PATH1, PATH2 = path_gen(cur_pos, destination, cur_theta, dest_angle, 7, 2*l_2)
            dest_1 = PATH1[-1]
            dest_2 = PATH2[-1]

            fi_1 = worldeu_2_mapeu(math.atan2((dest_1[1]-cur_car1[1]),dest_1[0]-cur_car1[0]))
            fi_2 = worldeu_2_mapeu(math.atan2((dest_2[1]-cur_car2[1]),dest_2[0]-cur_car2[0]))

            print(fi_1,fi_2)

            while ((abs(fi1_cur - fi_1) > 0.02) &  (abs(fi1_cur - fi_1) < math.pi*2-0.02)) | ((abs(fi2_cur - fi_2)> 0.02 ) & (abs(fi2_cur - fi_2) < math.pi*2-0.02)):
                yaw1 = 0.0
                yaw2 = 0.0
                if ((fi1_cur - fi_1 > 0.02) & (fi1_cur - fi_1 < math.pi)) | ((fi1_cur - fi_1 < -math.pi+0.02)) :
                    yaw1=yaw_max
                elif ((fi1_cur - fi_1 < -0.02)& (fi1_cur - fi_1 > -math.pi)) | ((fi1_cur - fi_1 > math.pi-0.02)) :
                    yaw1=-yaw_max

                if ((fi2_cur - fi_2 > 0.02) & (fi2_cur - fi_2 < math.pi)) | ((fi2_cur - fi_2 < -math.pi+0.02)) :
                    yaw2=yaw_max
                elif ((fi2_cur - fi_2 < -0.02)& (fi2_cur - fi_2 > -math.pi)) | ((fi2_cur - fi_2 > math.pi-0.02)) :
                    yaw2=-yaw_max

                car1.husky_cmd(0, yaw1)
                car2.husky_cmd(0, yaw2)

                rospy.sleep(duration)

                pose1 = car1.pose_get()
                pose2 = car2.pose_get()

                fi1_cur = worldeu_2_mapeu(quat2eu(pose1.orientation.z, pose1.orientation.w))
                fi2_cur = worldeu_2_mapeu(quat2eu(pose2.orientation.z, pose2.orientation.w))
                print(fi1_cur,fi2_cur)

            
            
            # dis1=norm(cur_car1, dest_1)
            # dis2=norm(cur_car2, dest_2)
            dis1=0
            dis2=0
            for cc in range(1,len(PATH1)):
                dis1 += norm(PATH1[cc-1],PATH1[cc])
                dis2 += norm(PATH2[cc-1],PATH2[cc])

            print(dis1, dis2)
            maximun = max(dis1, dis2)

            controller1 = PurePursuit(PATH1)
            controller2 = PurePursuit(PATH2)

            pose1 = car1.pose_get()
            pose2 = car2.pose_get()
            cur_car1 = np.array([pose1.position.x, pose1.position.y])
            cur_car2 = np.array([pose2.position.x, pose2.position.y])

            while norm(destination,cur_pos) > 0.5 :
                v1_out=0
                v2_out=0
                omega1=0
                omega2=0

                if norm(dest_1, cur_car1)>0.5:
                    target1 = controller1.search_target(car1.pose_get())
                    v1_out = vel_max * dis1/maximun

                if norm(dest_2, cur_car2)>0.5:
                    target2 = controller2.search_target(car2.pose_get())
                    v2_out = vel_max * dis2/maximun

                if (v1_out==0) & (v2_out==0):
                    break


                omega1 = pure_pursuit_control(car1.pose_get(), target1, v1_out)
                omega2 = pure_pursuit_control(car2.pose_get(), target2, v2_out)
                car1.husky_cmd(v1_out, omega1)
                car2.husky_cmd(v2_out, omega2)

                rospy.sleep(0.02)

                pose1 = car1.pose_get()
                pose2 = car2.pose_get()
                cur_car1 = np.array([pose1.position.x, pose1.position.y])
                cur_car2 = np.array([pose2.position.x, pose2.position.y])
                cur_pos = (cur_car1+cur_car2)/2


    else:
        print(flag1, flag2, "起点/终点设置错误！-- G0 ")
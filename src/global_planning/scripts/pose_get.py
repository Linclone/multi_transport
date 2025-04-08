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
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

import util

random.seed(2025) # for testing

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


confs = Base_confi()
O_x=float(confs.config_get('PLANNING','O_x'))
O_y=float(confs.config_get('PLANNING','O_y'))







class husky_ctrl:
    def __init__(self, robot_name, cmd_name):
        self.begin = 1
        self.robot_name = robot_name
        self.cmd_name = cmd_name
        self.cmd_twist = Twist()
        self.real_pose = Pose()
        self.cmdtwist_pub = rospy.Publisher(self.cmd_name, Twist, queue_size=5)
        print("send command to ", self.cmd_name)
        self.pose_sub = rospy.Subscriber('/vrpn_client_node/'+self.robot_name+'/pose',PoseStamped,self.state_callback)
        print('receive from /vrpn_client_node/'+self.robot_name+'/pose')

    def state_callback(self, data):
        self.real_pose = data.pose
        self.real_pose.position.x -= O_x
        self.real_pose.position.y -= O_y
        self.begin=0

    def husky_cmd(self, x, z):
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
        self.cmdtwist_pub.publish(self.cmd_twist)

    def pose_get(self):
        while self.begin:
            pass
        return self.real_pose

    def twist_get(self):
        return self.real_twist



def norm(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    


def gazebo2pgm(pgmsize, realmap_size, P):
    # 仅正方形
    return np.array([pgmsize*0.5+P[0]*pgmsize/realmap_size, pgmsize*0.5-P[1]*pgmsize/realmap_size])

def pgm2gazebo(pgmsize, realmap_size, Q):
    return np.array([-(realmap_size/pgmsize*(pgmsize*0.5-Q[0])),(realmap_size/pgmsize*(pgmsize*0.5-Q[1]))])
        
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



if __name__ == '__main__':

    rospy.init_node("poseget")

    rospack = rospkg.RosPack()

    # 获取当前包的路径
    package_path = rospack.get_path('global_planning')
    path=package_path + '/map/realworld.pgm'
    


    
    map=util.read_pgm(path)
    print('\n-----using map in', path, ', mapshape = ', map.shape)
    
    car1 = husky_ctrl('car1','cmd_vel0')
    car2 = husky_ctrl('car2','cmd_vel')

    vel_max = float(confs.config_get('PLANNING','Vm'))
    yaw_max = float(confs.config_get('PLANNING','Yawm'))
    pose1 = car1.pose_get()
    pose2 = car2.pose_get()

    cur_car1 = np.array([pose1.position.x, pose1.position.y])
    cur_car2 = np.array([pose2.position.x, pose2.position.y])

    fi_car1 = worldeu_2_mapeu(quat2eu(pose1.orientation.z, pose1.orientation.w))
    fi_car2 = worldeu_2_mapeu(quat2eu(pose2.orientation.z, pose2.orientation.w))

    l_2 = 0.5*norm( cur_car1, cur_car2 ) # 小车距中心的像素距离，即两小车距离的一半

    print("car1 position = ",cur_car1,"; orientation =",fi_car1)
    print("car2 position = ",cur_car2,"; orientation =",fi_car2)
    print("l = ",l_2*2, '(m)')


    S=np.array(gazebo2pgm(600,9,[cur_car1[0],cur_car1[1]])).astype(int)
    E=np.array(gazebo2pgm(600,9,[cur_car2[0],cur_car2[1]])).astype(int)

    path=[S,E]
    for i in range(0,len(path)):
        for j in range(-5,5+1):
            map[int(round(path[i][1]+j)),int(round(path[i][0]+j))]=0
        for j in range(-5,5+1):
            map[int(round(path[i][1]))+j,int(round(path[i][0]))-j]=0

    
    plt.imshow(map,cmap="gray", vmin=0,vmax=1)
    plt.show()

    
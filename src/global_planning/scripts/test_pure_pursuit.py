#!/usr/bin/env python

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

def norm(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


def gazebo2pgm(pgmsize, realmap_size, P):
    # 仅正方形
    return np.array([pgmsize*0.5+P[0]*pgmsize/realmap_size, pgmsize*0.5-P[1]*pgmsize/realmap_size])

def pgm2gazebo(pgmsize, realmap_size, Q):
    return np.array([-int(realmap_size/pgmsize*(pgmsize*0.5-Q[0])),int(realmap_size/pgmsize*(pgmsize*0.5-Q[1]))])

def quat2eu(z,w):
    # -3.14 ~ 3.14 , 0 指向pgm图像左侧，上半为正
    eu = math.atan2(2 * w*z , 1 - 2 * z*z)
    return (eu)
        

def worldeu_2_mapeu(eu):
    if eu>=0:
        out = math.pi - eu
    else:
        out = - math.pi - eu
    return out

Vmax=0.7

class PurePursuit:
    def __init__(self, path):
        self.path = path
        self.lookahead_distance = 3.0  # 预瞄距离
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

def generate_ellipse_path(a=6, b=3, num=50):
    theta = np.linspace(0, np.pi, num)
    x = a * np.cos(theta)
    y = b * np.sin(theta)
    return list(zip(x, y))

def pure_pursuit_control(pose, target):
    alpha = np.arctan2(target[1] - pose.position.y, target[0] - pose.position.x) - quat2eu(pose.orientation.z, pose.orientation.w)
    omega = 2 * Vmax * np.sin(alpha) / np.hypot(target[0]-pose.position.x, target[1]-pose.position.y)
    return omega



rospy.init_node("global_planning")
car = husky_ctrl('husky_1')
# 生成椭圆路径
path = generate_ellipse_path()

# 初始化车辆和控制器
controller = PurePursuit(path)
while(norm([car.pose_get().position.x, car.pose_get().position.y], path[-1]) > 0.5):
    target = controller.search_target(car.pose_get())
    omega = pure_pursuit_control(car.pose_get(), target)
    car.husky_cmd(Vmax, omega)
    rospy.sleep(0.02)

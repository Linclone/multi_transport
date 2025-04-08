#!/usr/bin/env python

import rospy
import rospkg

import math
import numpy as np

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates

import util

def norm(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

class husky_ctrl:
    def __init__(self, robot_name):
        self.begin = 1
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.real_pose = Pose()
        self.real_twist = Twist()
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

    def pose_get(self):
        return self.real_pose

    def twist_get(self):
        return self.real_twist


if __name__ == '__main__':

    rospy.init_node("measure")

    car1 = husky_ctrl('husky_1')
    car2 = husky_ctrl('husky_2')
    while(True):


        pose1 = car1.pose_get()
        pose2 = car2.pose_get()

        cur_car1 = np.array([pose1.position.x, pose1.position.y])
        cur_car2 = np.array([pose2.position.x, pose2.position.y])
        
        print(norm(cur_car1, cur_car2))

        rospy.sleep(0.2)
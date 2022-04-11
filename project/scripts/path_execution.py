#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from math import *
import numpy as np
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty, String
from transform import Transform
from action import Crazyflie


class PathExecution:

    def __init__(self):
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

        self.current_pose = None
        self.tf = Transform()
        self.cf = Crazyflie()

        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        self.rate = rospy.Rate(20)

    def execute_path(self, setpoints):
        tol_pos = 0.05
        tol_rot = 10
        # print("*************** New path ***************")
        # print("start pose: ",self.tf.position_msg(self.current_pose))
        # print("****************************************")
        for setpoint in setpoints:
            goal_pose = self.tf.transform2odom(setpoint)
            goal_pose.header.seq = 0
            # print(goal_pose)
            # print("****************************************")
            if goal_pose:
                #if abs(np.mod((goal_pose.yaw - np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation)) + 180), 360) - 180) > tol_rot:
                self.cf.rotate(goal_pose.yaw)
                self.cf.goTo(goal_pose)

    def pose_callback(self, msg):
        self.current_pose = msg

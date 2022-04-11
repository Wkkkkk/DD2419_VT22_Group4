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

    def execute_path(self, path):
        tol_pos = 0.05
        tol_rot = 10
        print("*************** New path ***************")
        print("start pose: ",self.tf.position_msg(self.current_pose))
        print("****************************************")
        for setpoint in path.poses:
<<<<<<< HEAD
            goal_pose = self.tf.transform2odom(setpoint)
            goal_pose.header.seq = 0
            print(goal_pose)
            print("****************************************")
            if goal_pose:
                if abs(np.mod((goal_pose.yaw - np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation)) + 180), 360) - 180) > tol_rot:
                    self.cf.rotate(goal_pose.yaw, 50)
                self.cf.goTo(goal_pose)
=======
            odom_point = self.tf.transform2odom(setpoint)
            odom_point.header.seq = 0
            print(odom_point)
            #print("****************************************")
            if odom_point:
                if abs(odom_point.yaw - np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation))) > tol_rot:
                    delta_yaw = tol_rot
                    goal_yaw = odom_point.yaw
                    yaw = np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation))
                    if yaw > goal_yaw and abs(yaw-goal_yaw) < 180:
                        delta_yaw = -delta_yaw
                    #N = int(abs(goal_yaw-yaw)/abs(delta_yaw))
                    #print("goal yaw: ", goal_yaw)
                    #print("current yaw: ", yaw)
                    while not rospy.is_shutdown():
                        yaw += delta_yaw
                        #print("piecewise yaw: ", yaw)
                        if yaw > 180:
                            yaw = yaw - 360
                        if yaw < -180:
                            yaw = yaw + 360
                        odom_point.yaw = yaw
                        odom_point.header.stamp = rospy.Time.now()
                        count = 0
                        while count < 2:
                            self.pub_cmd.publish(odom_point)
                            self.rate.sleep()
                            count += 1

                        if abs(yaw - goal_yaw) < tol_rot:
                            if abs(goal_yaw - np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation))) > tol_rot:
                                odom_point.yaw = goal_yaw
                                while not rospy.is_shutdown():
                                    odom_point.header.seq += 1
                                    odom_point.header.stamp = rospy.Time.now()
                                    self.pub_cmd.publish(odom_point)
                                    self.rate.sleep()
                                    rot_diff = abs(yaw - np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation)))
                                    print("rot diff: ",rot_diff)
                                    if rot_diff < tol_rot:
                                        break
                            break

                else:
                    #self.cf.goTo(odom_point)
                    #if (odom_point.x - self.current_pose.pose.position.x) ** 2 + (odom_point.y - self.current_pose.pose.position.y) ** 2 > tol_pos:
                    while not rospy.is_shutdown():
                        # rot_diff = abs(odom_point.yaw-degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation)))
                        # print("pos diff",pos_diff)
                        # print("rot diff",rot_diff)
                        odom_point.header.seq += 1
                        odom_point.header.stamp = rospy.Time.now()
                        self.pub_cmd.publish(odom_point)
                        self.rate.sleep()
                        pos_diff = (odom_point.x - self.current_pose.pose.position.x) ** 2 + (odom_point.y - self.current_pose.pose.position.y) ** 2
                        # if pos_diff < tol_pos and rot_diff < tol_rot:
                        if pos_diff < tol_pos:
                            break


    """def path_callback(self, msg):
        rospy.loginfo("Executing the trajectory!")
        self.path = msg
        self.publish_flag("executing")
        self.execute_path()
        rospy.loginfo("Finished  the trajectory!")"""
>>>>>>> 81ad26e3db127fe6c2ace5605edf0e71a977959c

    def pose_callback(self, msg):
        self.current_pose = msg

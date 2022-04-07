#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from math import *
import numpy as np
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty
from transform import Transform


class PathExecution:

    def __init__(self):
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
        self.pub_executed = rospy.Publisher('/mission_planner/is_executed', Bool, queue_size=1)

        self.new_path = False
        self.path = None
        self.current_pose = None
        self.tr = Transform()

        self.sub_path = rospy.Subscriber('/mission_planner/path', Path, self.path_callback)
        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        self.rate = rospy.Rate(10)

        #self.execute_path()

    def execute_path(self):
        self.publish_flag(False)
        tol_pos = 0.1
        tol_rot = 5
        print("len:", len(self.path.poses))
        for setpoint in self.path.poses:
            self.rate.sleep()
            odom_point = self.tr.transform2odom(setpoint)
            if odom_point:
                while not rospy.is_shutdown():
                    pos_diff = (odom_point.x - self.current_pose.pose.position.x) ** 2 + (odom_point.y - self.current_pose.pose.position.y) ** 2
                    rot_diff = abs(odom_point.yaw-self.tr.quaternion2yaw(self.current_pose.pose.orientation))
                    if pos_diff < tol_pos and rot_diff < tol_rot:
                        break
                    self.pub_cmd.publish(odom_point)
                    self.rate.sleep()

        self.publish_flag(True)

    def path_callback(self, msg):
        rospy.loginfo("Executing the trajectory!")
        self.path = msg
        self.execute_path()
        rospy.loginfo("Finished  the trajectory!")

    def pose_callback(self, msg):
        self.current_pose = msg

    def publish_flag(self, bool):
        msg = Bool()
        msg.data = bool
        self.pub_executed.publish(msg)


if __name__ == "__main__":
    rospy.init_node("path_execution")

    PathExecution()

    rospy.spin()


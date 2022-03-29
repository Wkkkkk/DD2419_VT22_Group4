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


class PathExecution:

    def __init__(self):
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
        #self.pub_executed = rospy.Publisher('/mission_planner/path_execution', String, queue_size=1)

        self.new_path = False
        #self.path = None
        self.current_pose = None
        self.tr = Transform()

        #self.sub_path = rospy.Subscriber('/mission_planner/path', Path, self.path_callback)
        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        self.rate = rospy.Rate(10)

        #self.execute_path()

    def execute_path(self, path):
        tol_pos = 0.05
        tol_rot = 10
        print("len:", len(path.poses))
        for setpoint in path.poses:
            #self.rate.sleep()
            odom_point = self.tr.transform2odom(setpoint)
            if odom_point:
                while not rospy.is_shutdown():
                    pos_diff = (odom_point.x - self.current_pose.pose.position.x) ** 2 + (odom_point.y - self.current_pose.pose.position.y) ** 2
                    # rot_diff = abs(odom_point.yaw-degrees(self.tr.quaternion2yaw(self.current_pose.pose.orientation)))
                    # print("pos diff",pos_diff)
                    # print("rot diff",rot_diff)
                    self.pub_cmd.publish(odom_point)
                    self.rate.sleep()
                    # if pos_diff < tol_pos and rot_diff < tol_rot:
                    if pos_diff < tol_pos:
                        break

        #self.publish_flag("finished")

    """def path_callback(self, msg):
        rospy.loginfo("Executing the trajectory!")
        self.path = msg
        self.publish_flag("executing")
        self.execute_path()
        rospy.loginfo("Finished  the trajectory!")"""

    def pose_callback(self, msg):
        self.current_pose = msg

    """def publish_flag(self, string):
        msg = String()
        msg.data = string
        self.pub_executed.publish(msg)"""


"""if __name__ == "__main__":
    rospy.init_node("path_execution")

    PathExecution()

    rospy.spin()"""
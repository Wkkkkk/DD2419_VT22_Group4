#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from math import *
import numpy as np
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from stds_msgs.msg import Bool, Empty
from transform import Transform as tr


def path_callback(msg):
    global path, new_path
    path = msg
    new_path = True


def pose_callback(msg):
    global current_pose
    current_pose = msg


def localized_callback(msg):
    global localized
    localized = msg


def publish_flag(bool):
    msg = Bool()
    msg.data = bool
    pub_executed.publish(msg)

def execute_path():
    #running_stamp = path.header.stamp
    tol_pos = 0.1
    tol_rot = 5
    for setpoint in path.poses:
        rate.sleep()
        odom_point = tr.transform2odom(setpoint)
        if odom_point:
            while not rospy.is_shutdown():
                pos_diff = (odom_point.x - current_pose.pose.position.x) ** 2 + (odom_point.y - current_pose.pose.position.y) ** 2
                rot_diff = abs(odom_point.yaw-tr.quaternion2yaw(current_pose.pose.orientation))
                if pos_diff < tol_pos and rot_diff < tol_rot:
                    while not localized:
                        pub_cmd.publish(odom_point)
                    break
                pub_cmd.publish(odom_point)
                rate.sleep()

    publish_flag(True)
    """goal = odom_point
    while running_stamp == path.header.stamp and not rospy.is_shutdown():
        pub.publish(goal)"""


def main():
    global new_path
    publish_flag(False)
    while not rospy.is_shutdown():
        if new_path:
            new_path = False
            execute_path()


if __name__ == "__main__":
    rospy.init_node("path_execution")

    pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    pub_executed = rospy.Publisher('/mission_planner/is_executed', Empty, queue_size=1)

    new_path = False
    path = None
    current_pose = None
    localized = False

    rospy.Subscriber('/mission_planner/path', Path, path_callback)
    rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/is_initialized', Bool, localized_callback)

    rate = rospy.Rate(10)

    main()

    rospy.spin()

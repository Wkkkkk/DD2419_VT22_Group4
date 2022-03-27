#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from math import *
import numpy as np
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from stds_msgs.msg import Bool, String
from transform import Transform as tr


def pose_callback(msg):
    global current_pose
    current_pose = msg


def localized_callback(msg):
    global localized
    localized = msg


def motion_callback(msg):
    motion = msg.data
    if motion == "rotate":
        rotate()
    if motion == "hover":
        hover()
    if motion == "liftoff":
        liftoff()
    if motion == "land":
        land()


def rotate():
    return 0


def hover():
    return 0


def liftoff():
    return 0


def land():
    return 0


if __name__ == "__main__":
    rospy.init_node("play_motion")

    pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

    current_pose = None
    localized = False

    rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/motion', String, motion_callback)
    rospy.Subscriber('/is_initialized', Bool, localized_callback)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10)

    rospy.spin()

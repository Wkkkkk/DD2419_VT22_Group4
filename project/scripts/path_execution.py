#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from math import *
import numpy as np
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
from stds_msgs.msg import Bool


def path_callback(msg):
    global path
    path = msg


def pose_callback(msg):
    global current_pose
    current_pose = msg


def localized_callback(msg):
    global localized
    localized = msg


def transform2odom(m):
    timeout = rospy.Duration(0.5)
    if not tf_buffer.can_transform(m.header.frame_id, 'cf1/odom', m.header.stamp, timeout):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % m.header.frame_id)
        return

    goal_odom = tf_buffer.transform(m, 'cf1/odom')

    goal = Position()
    goal.header.stamp = m.header.stamp
    goal.x = goal_odom.pose.position.x
    goal.y = goal_odom.pose.position.y
    goal.z = goal_odom.pose.position.z
    goal.header.frame_id = 'cf1/odom'
    roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
                                              goal_odom.pose.orientation.y,
                                              goal_odom.pose.orientation.z,
                                              goal_odom.pose.orientation.w))
    goal.yaw = degrees(yaw)
    return goal


def get_yaw(q):
    return degrees(atan2(2 * (q.w * q.z + q.x * q.y),
                 1 - 2 * (q.y * q.y + q.z * q.z)))


def execute_path():
    running_stamp = path.header.stamp
    tol_pos = 0.1
    tol_rot = 5
    for setpoint in path.poses:
        rate.sleep()
        odom_point = transform2odom(setpoint)
        if odom_point:
            while not rospy.is_shutdown():
                pos_diff = (odom_point.x - current_pose.pose.position.x) ** 2 + (odom_point.y - current_pose.pose.position.y) ** 2
                rot_diff = abs(odom_point.yaw-get_yaw(current_pose.pose.orientation))
                if pos_diff < tol_pos and rot_diff < tol_rot:
                    while not localized:
                        pub.publish(odom_point)
                    break
                pub.publish(odom_point)
                rate.sleep()

    goal = odom_point
    while running_stamp == path.header.stamp and not rospy.is_shutdown():
        pub.publish(goal)


def main():

    while not rospy.is_shutdown():
        if path:
            execute_path()


if __name__ == "__main__":
    rospy.init_node("path_execution")

    pub = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

    path = None
    current_pose = None
    localized = False

    rospy.Subscriber('/is_initialized', Bool, localized_callback)
    rospy.Subscriber('/planner/path', Path, path_callback)
    rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)

    tf_buffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10)

    main()

    rospy.spin()

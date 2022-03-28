#!/usr/bin/env python

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import rospy
from math import *

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)


class Transform:

    def transform2map(self, m):
        """ Transforms the drone pose from odom frame to map frame """
        timeout = rospy.Duration(0.5)
        if not tf_buffer.can_transform(m.header.frame_id, 'map', m.header.stamp, timeout):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % m.header.frame_id)
            return

        goal_map = tf_buffer.transform(m, 'map')
        goal_map.header.frame_id = 'map'
        goal_map.header.stamp = m.header.stamp

        return goal_map

    def transform2odom(self, m):
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

    def quaternion2yaw(self, q):
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def yaw2quaternion(self, yaw):
        return quaternion_from_euler(0.0, 0.0, yaw)

    def pose_stamped(self, position, yaw):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        q = self.yaw2quaternion(yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        msg.header.frame_id = 'map'
        return msg
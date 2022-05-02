#!/usr/bin/env python

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import rospy
import numpy as np


class Transform:
    """ Class used to do all sorts of transforms required for the mission planning """
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform2map(self, m):
        """ Transforms the drone pose from odom frame to map frame """
        timeout = rospy.Duration(0.5)
        if not self.tf_buffer.can_transform(m.header.frame_id, 'map', m.header.stamp, timeout):
            rospy.logwarn_throttle(5.0, 'No transform from %s to map' % m.header.frame_id)
            return

        goal_map = self.tf_buffer.transform(m, 'map')
        goal_map.header.frame_id = 'map'
        goal_map.header.stamp = m.header.stamp

        return goal_map

    def transform2odom(self, m):
        """ Transforms the drone pose from map frame to odom frame """
        timeout = rospy.Duration(0.5)
        if not self.tf_buffer.can_transform(m.header.frame_id, 'cf1/odom', m.header.stamp, timeout):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % m.header.frame_id)
            return

        goal_odom = self.tf_buffer.transform(m, 'cf1/odom')

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
        goal.yaw = np.rad2deg(yaw)
        return goal

    def quaternion2yaw(self, q):
        """ Transform quaternion to yaw """
        roll, pitch, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        return np.rad2deg(yaw)

    def yaw2quaternion(self, yaw):
        """ Transform yaw to quaternion """
        return quaternion_from_euler(0.0, 0.0, np.deg2rad(yaw))

    def pose_stamped_msg(self, position, yaw):
        """ Create a PoseStamped message from 3D position and yaw of the drone """

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        q = self.yaw2quaternion(yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def position_msg(self, m):
        """ Create a Position message from a PoseStamped message """
        msg = Position()
        msg.header.frame_id = 'cf1/odom'
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = 0
        msg.x = m.pose.position.x
        msg.y = m.pose.position.y
        msg.z = m.pose.position.z
        msg.yaw = np.rad2deg(self.quaternion2yaw(m.pose.orientation))
        return msg

    def posestamped_to_array(self, m):
        x = m.pose.position.x
        y = m.pose.position.y
        z = m.pose.position.z
        return np.array([x, y, z])

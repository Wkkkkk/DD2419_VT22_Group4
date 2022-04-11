#!/usr/bin/env python

import math
import numpy as np
import rospy
from crazyflie_driver.msg import Hover, Position
from std_msgs.msg import Empty
from threading import Thread
from transform import Transform
from geometry_msgs.msg import PoseStamped, Point


class Crazyflie:
    def __init__(self, prefix="cf1"):
        self.prefix = prefix
        self.tf = Transform()
        self.height = 0.4
        self.current_pose = None

        self.rate = rospy.Rate(10)

        self.sub = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        self.pub_hover = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.hover_msg = Hover()
        self.hover_msg.header.seq = 0
        self.hover_msg.header.stamp = rospy.Time.now()
        self.hover_msg.header.frame_id = 'c1/odom'
        self.hover_msg.yawrate = 0

        self.pub_position = rospy.Publisher(prefix + "/cmd_position", Position, queue_size=1)
        self.position_msg = Position()
        self.position_msg.header.seq = 0
        self.position_msg.header.frame_id = 'c1/odom'
        self.position_msg.header.stamp = rospy.Time.now()

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

        self.hover_timer = None


    def goTo(self, goal, vel = 0.2):
        start_pose = self.current_pose

        pos_tol = 0.05
        dt = 0.1
        self.position_msg.yaw = np.degrees(self.tf.quaternion2yaw(start_pose.pose.orientation))
        self.position_msg.header.seq = 0

        pose_array = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])
        goal_array = np.array([goal.x, goal.y, goal.z])
        while not rospy.is_shutdown() and np.linalg.norm(goal_array - pose_array) > pos_tol: 
            diff = goal_array - pose_array
            norm_diff = diff/np.linalg.norm(diff)
            pose_array += dt*vel * norm_diff
            self.position_msg.x = pose_array[0]
            self.position_msg.y = pose_array[1]
            self.position_msg.z = pose_array[2]
            self.position_msg.header.stamp = rospy.Time.now()
            self.position_msg.header.seq += 1
            self.pub_position.publish(self.position_msg)
            rospy.sleep(dt)

        start = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if (now - start > 2):
                break
            goal.header.stamp = rospy.Time.now()
            goal.header.seq += 1
            self.pub_position.publish(goal)
            self.rate.sleep()


    def start_hovering(self):
        rospy.loginfo("start hovering")
        self.position_msg = self.tf.position_msg(self.current_pose)
        self.hover_timer = rospy.Timer(rospy.Duration(1.0 / 20), self.hover)


    def stop_hovering(self):
        if self.hover_timer and self.hover_timer.is_alive():
            rospy.loginfo("stop hovering")
            self.hover_timer.shutdown()
            rospy.sleep(0.1)


    def hover(self, timer=None):
        self.position_msg.header.stamp = rospy.Time.now()
        self.position_msg.header.seq += 1
        self.pub_position.publish(self.position_msg)
        self.rate.sleep()


    # take off to height
    def takeOff(self, goal_height): 
        start_pose = self.current_pose

        self.position_msg.x = start_pose.pose.position.x
        self.position_msg.y = start_pose.pose.position.y
        #self.position_msg.z = 0.1
        self.position_msg.yaw = np.degrees(self.tf.quaternion2yaw(start_pose.pose.orientation))
        self.position_msg.header.seq = 0
        #self.pub_position.publish(self.position_msg)
        #self.rate.sleep()

        height = start_pose.pose.position.z
        tol = 0.05
        dt = 0.05
        vel = 0.35
        while not rospy.is_shutdown() and (goal_height - self.current_pose.pose.position.z) > tol:
            height_diff = goal_height - height
            height += dt*vel*height_diff
            self.position_msg.z = height
            self.position_msg.header.seq += 1
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            rospy.sleep(dt)

        start = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if (now - start > 2):
                break
            self.position_msg.z = goal_height
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            self.rate.sleep()


    # rotate itself
    def rotate(self, goal_yaw, yawrate=30):
        start_pose = self.current_pose
        yawrate = abs(yawrate)

        self.position_msg.x = start_pose.pose.position.x
        self.position_msg.y = start_pose.pose.position.y
        self.position_msg.z = start_pose.pose.position.z
        self.position_msg.header.seq = 0

        dt = 0.1
        yaw_tol = dt*abs(yawrate)

        yaw = np.degrees(self.tf.quaternion2yaw(start_pose.pose.orientation))
        while not rospy.is_shutdown() and abs(np.mod((goal_yaw - np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation)) + 180), 360) - 180) > yaw_tol:
            angular_diff = np.mod((goal_yaw - yaw + 180), 360) - 180
            yaw += dt*yawrate*np.sign(angular_diff)
            if abs(yaw) > 180:
                yaw += -np.sign(yaw)*360
            self.position_msg.yaw = yaw
            self.position_msg.header.seq += 1
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            rospy.sleep(dt)
        start = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if (now - start > 2):
                break
            self.position_msg.yaw = goal_yaw
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            self.rate.sleep()


    def land(self):
        start_pose = self.current_pose

        self.position_msg.x = start_pose.pose.position.x
        self.position_msg.y = start_pose.pose.position.y
        self.position_msg.yaw = np.degrees(self.tf.quaternion2yaw(start_pose.pose.orientation))
        self.position_msg.header.seq = 0

        height = start_pose.pose.position.z
        landing_height = 0.1
        tol = 0.05
        vel = 0.3
        dt = 0.1
        while not rospy.is_shutdown() and (self.current_pose.pose.position.z - landing_height) > tol:
            height_diff = landing_height - height
            height += dt*vel*height_diff
            self.position_msg.z = height
            self.position_msg.header.seq += 1
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            rospy.sleep(dt)
        self.stop_pub.publish(self.stop_msg)

    def pose_callback(self, msg):
        """ Retrieves the current pose of the drone in odom frame."""
        #self.current_pose = self.tf.transform2map(msg)
        self.current_pose = msg


if __name__ == '__main__':
    rospy.init_node('action', anonymous=True)

    cf = Crazyflie("cf1")

    while not rospy.is_shutdown() and cf.current_pose is None:
        continue

    cf.takeOff(0.5)

    # yaw = np.degrees(cf.tf.quaternion2yaw(cf.current_pose.pose.orientation))
    # for r in range(3):
    #     yaw += 90
    #     cf.rotate(yaw, 30)
    goal = Position()
    goal.x = cf.current_pose.pose.position.x + 1
    goal.y = cf.current_pose.pose.position.y
    goal.z = cf.current_pose.pose.position.z
    cf.goTo(goal)

    yaw = np.degrees(cf.tf.quaternion2yaw(cf.current_pose.pose.orientation))
    cf.rotate(yaw+180,30)
    goal.x = cf.current_pose.pose.position.x - 1
    goal.y = cf.current_pose.pose.position.y
    goal.z = cf.current_pose.pose.position.z
    cf.goTo(goal)
    cf.land()

    # goal = Position()
    # goal.x = 3.0
    # goal.y = -1
    # goal.z = 0.5
    # goal.yaw = 0

    # cf.goTo(goal)
    # cf.rotate(-100, 30)

    # goal.x = 1.5
    # goal.y = -1
    # cf.goTo(goal)

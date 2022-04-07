#!/usr/bin/env python

import math
import numpy as np
import rospy
from crazyflie_driver.msg import Hover, Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams, GoTo
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

        self.pub_position = rospy.Publisher(prefix + "/cmd_position", Position, queue_size=1)
        self.position_msg = None

        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

        # rospy.wait_for_service(prefix + '/update_params')
        # self.update_params = rospy.ServiceProxy(prefix + '/update_params', UpdateParams)        
        # self.setParam("commander/enHighLevel", 1)
        # rospy.wait_for_service(prefix + '/go_to')
        # self.goToService = rospy.ServiceProxy(prefix + "/go_to", GoTo)

        self.hover_timer = None

    # def setParam(self, name, value):
    #     rospy.set_param(self.prefix + "/" + name, value)
    #     self.update_params([name])

    def getSpeed(self, distance):
        if distance > 0:
            return 0.5
        elif distance < 0:
            return -0.5
        else:
            return 0

    def goTo(self, goal):
        current_position = self.tf.transform2odom(self.current_pose)
        x = goal.x - current_position.x
        y = goal.y - current_position.y
        duration = 0
        duration_x = 0
        duration_y = 0
        vx = 0
        vy = 0 

        # for x, in secs
        if x != 0:
            duration_x = abs(x/0.5)
            vx = self.getSpeed(x)

        # for y, in secs
        if y != 0:
            duration_y = abs(y/0.5)
            vy = self.getSpeed(y)

        durations = [duration_x, duration_y]
        duration = max(durations)

        if duration == 0:
            return
        elif duration == duration_x:
            vy *= abs(y/x)
        elif duration == duration_y:
            vx *= abs(x/y)

        start = rospy.get_time()
        while not rospy.is_shutdown():
            self.hover_msg.vx = vx
            self.hover_msg.vy = vy
            self.hover_msg.yawrate = 0.0
            self.hover_msg.zDistance = goal.z
            now = rospy.get_time()
            if (now - start > duration):
                break
            self.hover_msg.header.seq += 1
            self.hover_msg.header.stamp = rospy.Time.now()
            self.pub_hover.publish(self.hover_msg)
            self.rate.sleep()


        # gp = Point(goal.x, goal.y, goal.z)
        # self.goToService(0, False, gp, goal.yaw, rospy.Duration.from_sec(duration))
        # start = rospy.get_time()
        # while not rospy.is_shutdown():
        #     now = rospy.get_time()
        #     if (now - start > duration):
        #         break
        #     self.rate.sleep()


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
        self.pub_position.publish(self.position_msg)
        self.rate.sleep()

    # take off to height
    def takeOff(self, start_pose, height):
        self.position_msg = self.tf.position_msg(start_pose)
        self.position_msg.z = height
        self.pub_position.publish(self.position_msg)
        self.rate.sleep()
        # time_range = 1 + int(10*height/0.4)
        # while not rospy.is_shutdown():
        #     for y in range(time_range):
        #         self.hover_msg.vx = 0.0
        #         self.hover_msg.vy = 0.0
        #         self.hover_msg.yawrate = 0.0
        #         self.hover_msg.zDistance = y / 25.0
        #         self.hover_msg.header.seq += 1
        #         self.hover_msg.header.stamp = rospy.Time.now()
        #         self.pub_hover.publish(self.hover_msg)
        #         self.rate.sleep()
        #     for y in range(20):
        #         self.hover_msg.vx = 0.0
        #         self.hover_msg.vy = 0.0
        #         self.hover_msg.yawrate = 0.0
        #         self.hover_msg.zDistance = height
        #         self.hover_msg.header.seq += 1
        #         self.hover_msg.header.stamp = rospy.Time.now()
        #         self.pub_hover.publish(self.hover_msg)
        #         self.rate.sleep()
        #     break

    # rotate itself
    def rotate(self, height=0.4):
        rot_msg = self.tf.position_msg(self.current_pose)#self.tf.transform2odom(self.current_pose)
        yaw = rot_msg.yaw
        delta_yaw = 10
        #tol = 10
        count = 0
        while count < 360/delta_yaw-1:
            yaw += delta_yaw
            rot_msg.yaw = yaw
            rot_msg.header.stamp = rospy.Time.now()
            # if yaw >= 180:
            #     yaw = -360 + yaw
            # while abs(yaw-np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation))) > tol:
            #     print("1. ",yaw)
            #     print("2. ",np.degrees(self.tf.quaternion2yaw(self.current_pose.pose.orientation)))
            #     self.pub_position.publish(rot_msg)
            self.pub_position.publish(rot_msg)
            self.rate.sleep()
            count += 1

    def land(self):
        position_msg = self.tf.position_msg(self.current_pose)#self.tf.transform2odom(self.current_pose)
        landing_height = 0.1
        position_msg.z = landing_height
        while not self.current_pose.pose.position.z < landing_height:
            self.pub_position.publish(position_msg)
            self.rate.sleep()
        self.stop_pub.publish(self.stop_msg)

    def pose_callback(self, msg):
        """ Retrieves the current pose of the drone in odom frame."""
        #self.current_pose = self.tf.transform2map(msg)
        self.current_pose = msg

=======
        self.current_pose = self.tf.transform2map(msg)
>>>>>>> f80e47d14931fd563ed1f451a32c95f0d048f798

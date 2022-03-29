#!/usr/bin/env python

import rospy
import tf
from crazyflie_gazebo.msg import Hover
from std_msgs.msg import Empty
from crazyflie_ros.srv import Land, Takeoff, GoTo


class Crazyflie:
    def __init__(self, prefix="cf1"):
        self.prefix = prefix

        self.rate = rospy.Rate(10)
        self.duration = 5

        self.height = 0.4
        # Wait for service providers
        rospy.wait_for_service(prefix + '/land')
        rospy.wait_for_service(prefix + '/takeoff')
        #rospy.wait_for_service(prefix + '/go_to')

        self.pub_hover = rospy.Publisher(prefix + "/cmd_hover", Hover, queue_size=1)
        self.hover_msg = Hover()
        self.hover_msg.header.frame_id = 'cf1/odom'
        self.pub_stop = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()

        self.rotating = False

    # take off to height
    def takeOff(self, zDistance):
        self.height = zDistance
        rospy.loginfo("Taking off!")
        takeoff_client = rospy.ServiceProxy(self.prefix + '/takeoff', Takeoff)
        takeoff_client(0, self.height, rospy.Duration.from_sec(self.duration))

    def hover(self):
        self.hover_msg.vx = 0.0
        self.hover_msg.vy = 0.0
        self.hover_msg.yawrate = 0.0
        self.hover_msg.zDistance = self.height
        self.hover_msg.header.stamp = rospy.Time.now()
        self.pub_hover.publish(self.hover_msg)

    # rotate itself
    # yawrate [e.g. -200 to 200 degrees/second]
    # duration [seconds]
    def rotate(self, yawrate=10, max_duration=5):
        rospy.loginfo("Rotating")
        self.rotating = True
        timer = rospy.Timer(rospy.Duration(max_duration), self.stop_rotate)
   
        while not rospy.is_shutdown() and self.rotating:
            self.hover_msg.vx = 0.0
            self.hover_msg.vy = 0.0
            self.hover_msg.yawrate = yawrate
            self.hover.zDistance = self.height
            self.hover_msg.header.seq += 1
            self.hover_msg.header.stamp = rospy.Time.now()
            self.pub_hover.publish(self.hover_msg)
            self.rate.sleep()

    # stop rotating
    def stop_rotate (self, timer=None):
        rospy.loginfo("Stops rotating")
        self.rotating = False

    # land from last zDistance
    def land(self):
        rospy.loginfo("Begins to land!")
        land_client = rospy.ServiceProxy(self.prefix + '/land', Land)
        land_client(0, 0.1, self.duration)
        self.pub_stop.publish(self.stop_msg)



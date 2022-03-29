#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Hover, Position
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
from transform import Transform
from geometry_msgs.msg import PoseStamped


class Crazyflie:
    def __init__(self, prefix="cf1"):
        self.prefix = prefix
        self.tr = Transform()
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

        self.rotating = False
        self.rotate_timer = None
        self.hover_timer = None


    # determine direction of speed based on distance
    def getSpeed(self, distance):
        if distance > 0:
            return 0.1
        elif distance < 0:
            return -0.1
        else:
            return 0

    def start_hovering(self):
        rospy.loginfo("start hovering")
        # 20 hz
        self.hover_timer = rospy.Timer(rospy.Duration(1.0/20), self.hover)

    def stop_hovering(self):
        if self.hover_timer and self.hover_timer.is_alive():
            rospy.loginfo("stop hovering")
            self.hover_timer.shutdown()
            rospy.sleep(0.1)

    def hover(self, timer=None):
        #rospy.loginfo("hovering")
        """self.msg.vx = 0.0
        self.msg.vy = 0.0
        self.msg.yawrate = 0.0
        #self.msg.zDistance = self.msg.zDistance
        self.msg.header.seq += 1
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)"""

        self.pub_position.publish(self.position_msg)

        self.rate.sleep()

    # take off to z distance
    def takeOff(self, zDistance):
        time_range = 1 + int(10*zDistance/0.4)
        while not rospy.is_shutdown():
            for y in range(time_range):
                self.hover_msg.vx = 0.0
                self.hover_msg.vy = 0.0
                self.hover_msg.yawrate = 0.0
                self.hover_msg.zDistance = y / 25.0
                self.hover_msg.header.seq += 1
                self.hover_msg.header.stamp = rospy.Time.now()
                self.pub_hover.publish(self.hover_msg)
                self.rate.sleep()
            for y in range(20):
                self.hover_msg.vx = 0.0
                self.hover_msg.vy = 0.0
                self.hover_msg.yawrate = 0.0
                self.hover_msg.zDistance = zDistance
                self.hover_msg.header.seq += 1
                self.hover_msg.header.stamp = rospy.Time.now()
                self.pub_hover.publish(self.hover_msg)
                self.rate.sleep()

            self.position_msg = self.tr.transform2odom(self.current_pose)
            self.start_hovering()
            break


    # rotate itself
    # yawrate [e.g. -200 to 200 degrees/second]
    # duration [seconds]
    def rotate (self, yawrate=10, max_duration=5):
        print("rotating")
        self.rotating = True
        self.rotate_timer = rospy.Timer(rospy.Duration(max_duration), self.stop_rotate)
   
        while not rospy.is_shutdown() and self.rotating:
            self.hover_msg.vx = 0.0
            self.hover_msg.vy = 0.0
            self.hover_msg.yawrate = yawrate
            #self.msg.zDistance = self.msg.zDistance
            self.hover_msg.header.seq += 1
            self.hover_msg.header.stamp = rospy.Time.now()
            self.pub_hover.publish(self.hover_msg)
            self.rate.sleep()


    # stop rotating
    def stop_rotate (self, timer=None):    
        if self.rotate_timer and self.rotate_timer.is_alive():
            print("stop rotating")
            self.rotating = False
            self.rotate_timer.shutdown()


    # land from last zDistance
    def land (self):
        print("landing")
        # get last height
        zDistance = self.hover_msg.zDistance

        while not rospy.is_shutdown():
            while zDistance > 0:
                self.hover_msg.vx = 0.0
                self.hover_msg.vy = 0.0
                self.hover_msg.yawrate = 0.0
                self.hover_msg.zDistance = zDistance
                self.hover_msg.header.seq += 1
                self.hover_msg.header.stamp = rospy.Time.now()
                self.pub_hover.publish(self.hover_msg)
                self.rate.sleep()
                zDistance -= 0.05
        self.stop_pub.publish(self.stop_msg)

    def pose_callback(self, msg):
        """ Retrieves the current pose of the drone in odom frame."""
        self.current_pose = self.tr.transform2map(msg)

if __name__ == '__main__':
    rospy.init_node('hover', anonymous=True)
    cf = Crazyflie("cf1")

    cf.takeOff(0.4)
    cf.goTo(0, 1.5, 0.4, 0)
    cf.rotate(-15, 6)
    cf.start_hovering()
    rospy.sleep(1)
    cf.stop_hovering()
    cf.rotate(15, 6)
    cf.goTo(0, -1.5, 0.4, 0)
    cf.start_hovering()
    rospy.sleep(1)
    cf.stop_hovering()
    cf.land()
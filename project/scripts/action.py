#!/usr/bin/env python

import numpy as np
import rospy
from crazyflie_driver.msg import Hover, Position
from std_msgs.msg import Empty
from transform import Transform
from geometry_msgs.msg import PoseStamped, Point


class Crazyflie:
    def __init__(self, prefix="cf1"):

        self.prefix = prefix

        # Transform class object
        self.tf = Transform()

        # Initialize subscirber
        self.sub = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        # Initialize publishers
        self.pub_position = rospy.Publisher(prefix + "/cmd_position", Position, queue_size=1)
        self.stop_pub = rospy.Publisher(prefix + "/cmd_stop", Empty, queue_size=1)

        # Initialize messages
        self.position_msg = Position()
        self.position_msg.header.seq = 0
        self.position_msg.header.frame_id = 'c1/odom'
        self.position_msg.header.stamp = rospy.Time.now()

        self.stop_msg = Empty()

        # Initialize variables
        self.hover_timer = None
        self.current_pose = None
        self.rate = rospy.Rate(10)

    def goTo(self, goal, vel=0.25):
        """ Action to make drone fly straight to a set point at a velocity determined by vel """

        start_pose = self.current_pose

        pos_tol = 0.05  # Tolerance of difference in position
        dt = 0.1  # Rate at which to publish position command

        self.position_msg.yaw = self.tf.quaternion2yaw(start_pose.pose.orientation)
        self.position_msg.header.seq = 0

        position_array = np.array([start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z])
        goal_array = np.array([goal.x, goal.y, goal.z])

        while not rospy.is_shutdown() and np.linalg.norm(goal_array - position_array) > pos_tol:
            # Calculates the next position at dt seconds later
            diff = goal_array - position_array
            norm_diff = diff/np.linalg.norm(diff)
            position_array += dt*vel * norm_diff

            # Publish the next position
            self.position_msg.x = position_array[0]
            self.position_msg.y = position_array[1]
            self.position_msg.z = position_array[2]
            self.position_msg.header.stamp = rospy.Time.now()
            self.position_msg.header.seq += 1
            self.pub_position.publish(self.position_msg)

            rospy.sleep(dt)

        # Hover for a while
        start = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now - start > 2:
                break
            goal.header.stamp = rospy.Time.now()
            goal.header.seq += 1
            self.pub_position.publish(goal)
            self.rate.sleep()

    def start_hovering(self):
        """ Start hovering """
        rospy.loginfo("start hovering")
        self.position_msg = self.tf.position_msg(self.current_pose)
        self.hover_timer = rospy.Timer(rospy.Duration(1.0 / 20), self.hover)

    def stop_hovering(self):
        """ Stop hovering """
        if self.hover_timer and self.hover_timer.is_alive():
            rospy.loginfo("stop hovering")
            self.hover_timer.shutdown()
            rospy.sleep(0.1)

    def hover(self, timer=None):
        """ starts when start_hovering is called and stops when stop_hovering is called """
        self.position_msg.header.stamp = rospy.Time.now()
        self.position_msg.header.seq += 1
        self.pub_position.publish(self.position_msg)
        self.rate.sleep()

    def takeOff(self, goal_height):
        """ Make drone take off from the ground to a specified height """
        start_pose = self.current_pose

        self.position_msg.x = start_pose.pose.position.x
        self.position_msg.y = start_pose.pose.position.y
        self.position_msg.yaw = self.tf.quaternion2yaw(start_pose.pose.orientation)
        self.position_msg.header.seq = 0
        tol = 0.05

        # height = start_pose.pose.position.z
        # dt = 0.1
        # dz = 0.02
        # vel = 0.3
        # while not rospy.is_shutdown() and abs(goal_height - self.current_pose.pose.position.z) > tol:
        #     # Calculates the next height at dt seconds later
        #     #height_diff = goal_height - self.current_pose.pose.position.z
        #     #height += dt*vel*height_diff
        #     height += dz
        #     # Publish the next height
        #     self.position_msg.z = height
        #     self.position_msg.header.seq += 1
        #     self.position_msg.header.stamp = rospy.Time.now()
        #     self.pub_position.publish(self.position_msg)
        #     rospy.sleep(dt)


        for t in range(10):
            self.position_msg.z = t / 25
            self.position_msg.header.seq += 1
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            self.rate.sleep()
        while not rospy.is_shutdown() and abs(goal_height - self.current_pose.pose.position.z) > tol:
            # Publish the next height
            self.position_msg.z = goal_height
            self.position_msg.header.seq += 1
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            self.rate.sleep()


        # Hover for a while
        start = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now - start > 2:
                break
            self.position_msg.z = goal_height
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            self.rate.sleep()

    def rotate(self, goal_yaw, yawrate=30):
        """ Rotate the drone to a desired yaw at a rate determined by yawrate """

        start_pose = self.current_pose
        yawrate = abs(yawrate)

        self.position_msg.x = start_pose.pose.position.x
        self.position_msg.y = start_pose.pose.position.y
        self.position_msg.z = start_pose.pose.position.z
        self.position_msg.header.seq = 0

        dt = 0.1
        yaw_tol = dt*abs(yawrate)  # tolerance based on publishing rate and yaw rate

        yaw = self.tf.quaternion2yaw(start_pose.pose.orientation)
        while not rospy.is_shutdown() and self.yaw_difference(goal_yaw, yaw) > yaw_tol:
            # Calculates the next yaw at dt seconds later
            angular_diff = np.mod((goal_yaw - yaw + 180), 360) - 180
            yaw += dt*yawrate*np.sign(angular_diff)

            # Forces yaw to stay within [-180, 180] degrees
            if abs(yaw) > 180:
                yaw += -np.sign(yaw)*360

            # Publish the next yaw
            self.position_msg.yaw = yaw
            self.position_msg.header.seq += 1
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)

            rospy.sleep(dt)

        # Hover for a while
        start = rospy.get_time()
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if now - start > 2:
                break
            self.position_msg.yaw = goal_yaw
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)
            self.rate.sleep()

    def land(self):
        """ Lands the drone on the ground from its current height """

        start_pose = self.current_pose

        self.position_msg.x = start_pose.pose.position.x
        self.position_msg.y = start_pose.pose.position.y
        self.position_msg.yaw = self.tf.quaternion2yaw(start_pose.pose.orientation)
        self.position_msg.header.seq = 0

        landing_height = 0.1  # Height at which to stop the motors
        tol = 0.05
        vel = 0.3
        dt = 0.2
        dz = -0.02
        height = start_pose.pose.position.z
        while not rospy.is_shutdown() and abs(self.current_pose.pose.position.z - landing_height) > tol:
            # Calculates the next height at dt seconds later
            #height_diff = landing_height - height
            #height += dt*vel*height_diff
            height += dz
            # Publish the next height
            self.position_msg.z = height
            self.position_msg.header.seq += 1
            self.position_msg.header.stamp = rospy.Time.now()
            self.pub_position.publish(self.position_msg)

            rospy.sleep(dt)

        self.stop_pub.publish(self.stop_msg)  # Stop motors

    def yaw_difference(self, goal_yaw, yaw=None):
        """ Calculates difference between current yaw and goal yaw """
        if yaw is None:
            yaw = self.tf.quaternion2yaw(self.current_pose.pose.orientation)

        return abs(np.mod((goal_yaw - yaw + 180), 360) - 180)

    def pose_callback(self, msg):
        """ Retrieves the current pose of the drone in odometry frame """
        self.current_pose = msg


if __name__ == '__main__':
    rospy.init_node('action', anonymous=True)

    cf = Crazyflie("cf1")

    while not rospy.is_shutdown() and cf.current_pose is None:
        continue

    cf.takeOff(0.4)
    cf.land()

    # yaw = np.degrees(cf.tf.quaternion2yaw(cf.current_pose.pose.orientation))
    # for r in range(3):
    #     yaw += 90
    #     cf.rotate(yaw, 30)
    # goal = Position()
    # goal.x = cf.current_pose.pose.position.x + 1
    # goal.y = cf.current_pose.pose.position.y
    # goal.z = cf.current_pose.pose.position.z
    # cf.goTo(goal)

    # yaw = cf.tf.quaternion2yaw(cf.current_pose.pose.orientation)
    # cf.rotate(yaw+180,30)
    # goal.x = cf.current_pose.pose.position.x - 1
    # goal.y = cf.current_pose.pose.position.y
    # goal.z = cf.current_pose.pose.position.z
    # cf.goTo(goal)
    # print("Ready to land")
    # cf.land()

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


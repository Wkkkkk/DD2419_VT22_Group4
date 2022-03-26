#!/usr/bin/env python

import rospy
import math
import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import String
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
import timeit
from std_msgs.msg import Empty
import enum

from action import Crazyflie


# Using enum class create enumerations
class State(enum.Enum):
   Init = 1
   GenerateExplorationGoal = 2
   GoToExplorationGoal = 3
   RotateAndSearchForIntruder = 4
   Landing = 5


class StateMachine(object):

    def __init__(self):
        #self.rotate_srv_nm = rospy.get_param(rospy.get_name() + '/rotate_srv')
        #self.hover_srv_nm = rospy.get_param(rospy.get_name() + '/hover_srv')
        
        # Subscribe to topics
        sub_pose = rospy.Subscriber('cf1/pose', PoseStamped, self.pose_callback)

        # Wait for service providers
        #rospy.wait_for_service(self.rotate_srv_nm, timeout=3)
        #rospy.wait_for_service(self.hover_srv_nm)

        # Instantiate publishers
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

        # Init state machine
        self.goal = None
        self.current_pose = None
        self.cmd = 0
        self.state = State.Init
        self.cf = Crazyflie("cf1")

        # Wait to be initialized
        rospy.wait_for_message('is_initialized', Empty)

        self.states()


    def states(self):

        # State 0: lift off and hover
        if self.state == State.Init:
            self.cf.takeOff(0.4)
            self.state = State.GenerateExplorationGoal

        while not rospy.is_shutdown():
            # State 1: Generate next exploration goal from explorer
            if self.state == State.GenerateExplorationGoal:
                print("Generate goal")
                rospy.sleep(1)
                self.state = State.GoToExplorationGoal

            # State 2: Generate path to next exploration goal and execute it
            if self.state == State.GoToExplorationGoal:
                print("Go to goal")
                self.cf.goTo(0.4, 0.1, 0.2, 0)
                self.state = State.RotateAndSearchForIntruder

            # State 3: Rotate 90 degrees and hover a while three times while waiting for intruder detection
            if self.state == State.RotateAndSearchForIntruder:
                print("Check intruders")
                for _ in range(3):
                    self.cf.rotate(10, 5)
                self.state = State.Landing

            # State 4: Land on the ground when explorer can't find more space to explore
            if self.state == State.Landing:
                print("Finish task")
                break

        rospy.loginfo("%s: Tasks finished!")


    def pose_callback(self, msg):
        self.current_pose = msg


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
       StateMachine()
    except rospy.ROSInterruptException:
       pass

    rospy.spin()

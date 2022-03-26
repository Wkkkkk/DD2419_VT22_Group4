#!/usr/bin/env python

import rospy
import sys
import json
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
from grid_map import GridMap
from path_planner import Planner
from explore import Explore

# Using enum class create enumerations
class State(enum.Enum):
    Init = 1
    GenerateExplorationGoal = 2
    GoToExplorationGoal = 3
    RotateAndSearchForIntruder = 4
    Landing = 5


class StateMachine(object):
    def __init__(self, argv=sys.argv):
        # Initialize map
        args = rospy.myargv(argv=argv)
        with open(args[1], 'rb') as f:
            world = json.load(f)

        self.grid = GridMap(0.2, world)
        self.explore = Explore(self.grid)

        # Subscribe to topics
        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        # Instantiate publishers
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

        # Initialize state machine
        self.goal = None
        self.current_pose = None
        self.cmd = 0
        self.state = State.Init
        self.cf = Crazyflie("cf1")

        self.states()

    def states(self):
        next_pose = None

        # Wait for localization to be initialized
        rospy.wait_for_message('is_initialized', Empty)

        # State 1: lift off and hover
        if self.state == State.Init:
            self.cf.takeOff(0.4)
            self.state = State.GenerateExplorationGoal

        while not rospy.is_shutdown():
            # State 2: Generate next exploration goal from explorer
            if self.state == State.GenerateExplorationGoal:
                next_pose = self.explore.next_point()

                if next_pose is None:
                    self.state = State.Landing
                else:
                    self.state = State.GoToExplorationGoal

            # State 3: Generate path to next exploration goal and execute it
            if self.state == State.GoToExplorationGoal:
                print("Go to goal")
                # self.cf.goTo(0.4, 0.1, 0.2, 0)
                A = Planner(next_pose, self.grid)
                A.run()

                self.state = State.RotateAndSearchForIntruder

            # State 4: Rotate 90 degrees and hover a while while waiting for intruder detection
            if self.state == State.RotateAndSearchForIntruder:
                print("Check intruders")
                self.cf.rotate(10, 5)
                self.state = State.GenerateExplorationGoal

            # State 5: Land on the ground when explorer can't find more space to explore
            if self.state == State.Landing:
                print("Finish task")
                break

        rospy.loginfo("%s: State machine finished!")


    def pose_callback(self, msg):
        self.current_pose = msg


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

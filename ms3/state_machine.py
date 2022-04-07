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
from std_msgs.msg import Empty, Bool
import enum
from grid_map import GridMap
from path_planner import Planner
from explore import Explore
from action import Crazyflie
from transform import Transform


# Using enum class create enumerations
class State(enum.Enum):
    Init = 1
    GenerateExplorationGoal = 2
    GoToExplorationGoal = 3
    RotateAndSearchForIntruder = 4
    Landing = 5


class StateMachine(object):
    def __init__(self, argv=sys.argv):
        rospy.loginfo("Initializing state machine")
        # Initialize map
        args = rospy.myargv(argv=argv)
        with open(args[1], 'rb') as f:
            world = json.load(f)

        self.grid = GridMap(0.2, world)
        self.explore = Explore(self.grid)
        self.tr = Transform()
        self.tol = 0.05

        # Subscribe to topics
        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)
        self.sub_executed = rospy.Subscriber('/mission_planner/is_executed', Bool, self.path_executed_callback)

        # Instantiate publishers
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

        # Initialize state machine
        self.goal = None
        self.current_pose = None
        self.is_executed = False
        self.cmd = 0
        self.state = State.Init
        self.cf = Crazyflie("cf1")

        rospy.loginfo(" state machine initialzed")
        self.states()

    def states(self):
        next_pose = None

        # Wait for localization to be initialized
        #rospy.wait_for_message('is_initialized', Empty)
        rospy.loginfo("state machine ready")

        # State 1: lift off and hover
        if self.state == State.Init:
            height = 0.4
            self.wait_for_pose()
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = self.current_pose.pose.position.x
            goal_pose.pose.position.y = self.current_pose.pose.position.y
            goal_pose.pose.position.z = height
            self.cf.takeOff(height)
            rospy.loginfo("taken off")

            self.state = State.GenerateExplorationGoal
            #self.reach_goal(goal_pose)
            #rospy.loginfo("reached goal")

        while not rospy.is_shutdown():
            print("State:", self.state)

            # State 2: Generate next exploration goal from explorer
            if self.state == State.GenerateExplorationGoal:
                self.cf.start_hovering()
                rospy.loginfo("generating exploration")
                next_pose = self.explore.next_goal(self.current_pose)

                if next_pose is None:
                    self.state = State.Landing
                else:
                    self.state = State.GoToExplorationGoal

            # State 3: Generate path to next exploration goal and execute it
            if self.state == State.GoToExplorationGoal:
                rospy.loginfo("Go to next goal %s", next_pose)
                A = Planner(next_pose, self.grid)
                goal_found = A.run()
                if not goal_found:
                    self.state = State.Landing
                    continue

                self.cf.stop_hovering()
                while not self.is_executed:
                    continue
                self.state = State.RotateAndSearchForIntruder

            # State 4: Rotate 90 degrees and hover a while while waiting for intruder detection
            if self.state == State.RotateAndSearchForIntruder:
                rospy.loginfo("Checks for intruders")
                self.cf.rotate(10, 5)
                self.state = State.GenerateExplorationGoal

            # State 5: Land on the ground when explorer can't find more space to explore
            if self.state == State.Landing:
                self.cf.stop_hovering()
                self.cf.land()
                break

        rospy.loginfo("State machine finished!")

    def reach_goal(self, goal_pose):
        while not rospy.is_shutdown():
            diff = (goal_pose.pose.position.x - self.current_pose.pose.position.x) ** 2 + (goal_pose.pose.position.y - self.current_pose.pose.position.y) ** 2 \
            + (goal_pose.pose.position.z - self.current_pose.pose.position.z) ** 2
            if diff < self.tol:
                break

    def wait_for_pose(self):
        while not rospy.is_shutdown() and self.current_pose is None:
            continue

    def pose_callback(self, msg):
        self.current_pose = self.tr.transform2map(msg)


    def path_executed_callback(self, msg):
        self.is_executed = msg.data


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


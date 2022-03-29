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
from path_execution import PathExecution


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
        self.path_executer = PathExecution()
        self.tr = Transform()
        self.tol = 0.05
        self.rate = rospy.Rate(10)

        # Subscribe to topics
        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)
        #self.sub_executed = rospy.Subscriber('/mission_planner/path_execution', String, self.path_execution_callback)

        # Instantiate publishers
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

        # Initialize state machine
        self.goal = None
        self.current_pose = None
        self.path_execution = None
        self.cmd = 0
        self.state = State.Init
        self.cf = Crazyflie("cf1")

        rospy.loginfo("state machine initialzed")
        self.states()

    def states(self):
        next_pose = None

        # Wait for localization to be initialized
        #rospy.wait_for_message('is_initialized', Empty

        # State 1: lift off and hover
        if self.state == State.Init:
            height = 0.4
            self.wait_for_pose()
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = self.current_pose.pose.position.x
            goal_pose.pose.position.y = self.current_pose.pose.position.y
            goal_pose.pose.position.z = height
            rospy.loginfo("Taking off")
            self.cf.takeOff(height)
            self.cf.start_hovering()
            self.state = State.GenerateExplorationGoal
            #self.reach_goal(goal_pose)
            #rospy.loginfo("reached goal")

        while not rospy.is_shutdown():
            print("State:", self.state)
            #self.cf.start_hovering()
            # State 2: Generate next exploration goal from explorer
            if self.state == State.GenerateExplorationGoal:
                rospy.loginfo("Generating the next exploration goal")
                next_pose = self.explore.next_goal(self.current_pose)
                if next_pose is None:
                    self.state = State.Landing
                else:
                    self.state = State.GoToExplorationGoal

            # State 3: Generate path to next exploration goal and execute it
            if self.state == State.GoToExplorationGoal:
                rospy.loginfo("Go to next goal")
                A = Planner(next_pose, self.grid)
                path = A.run()
                if path is None:
                    self.state = State.GenerateExplorationGoal
                    #self.state = State.Landing
                    continue
                """while not self.path_execution == "executing":
                    continue"""
                self.cf.stop_hovering()
                """while not self.path_execution == "finished":
                    continue"""
                # print(path)
                self.path_executer.execute_path(path)
                self.state = State.RotateAndSearchForIntruder

            # State 4: Rotate 90 degrees and hover a while while waiting for intruder detection
            if self.state == State.RotateAndSearchForIntruder:
                rospy.loginfo("Checks for intruders")
                self.cf.rotate(3, 5)
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

    def path_execution_callback(self, msg):
        self.path_execution = msg.data


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
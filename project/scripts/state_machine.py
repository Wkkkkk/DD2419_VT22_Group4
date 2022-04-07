#!/usr/bin/env python

import sys
import json
import rospy

import tf2_ros
from std_msgs.msg import String
import tf2_geometry_msgs

from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
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
        self.path_executer = PathExecution()
        self.tf = Transform()
        self.cf = Crazyflie("cf1")

        self.current_pose = None 

        # Subscribe to topics
        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        self.wait_for_pose()
        self.start_pose = self.tf.transform2map(self.current_pose)
        self.explore = Explore(self.grid, self.start_pose)

        self.tol = 0.05
        self.rate = rospy.Rate(10)
        self.height = 0.4

        # Initialize state machine
        self.state = State.Init

        rospy.loginfo("state machine initialzed")
        self.states()

    def states(self):
        next_pose = None
        #self.wait_for_pose()
        # Wait for localization to be initialized
        # rospy.wait_for_message('is_initialized', Empty)
        rospy.loginfo("Taking off")
        while not rospy.is_shutdown():

            # State 1: lift off and hover
            if self.state == State.Init:
                self.cf.takeOff(self.start_pose, self.height)
                if self.tol > abs(self.current_pose.pose.position.z - self.height):
                    #self.cf.start_hovering()
                    self.state = State.RotateAndSearchForIntruder
                    # self.state = State.GenerateExplorationGoal

            # State 2: Generate next exploration goal from explorer
            if self.state == State.GenerateExplorationGoal:
                rospy.loginfo("Generating the next exploration goal")
                self.start_pose = self.tf.transform2map(self.current_pose)
                next_pose = self.explore.next_goal()
                if next_pose is None:
                    self.state = State.Landing
                else:
                    self.state = State.GoToExplorationGoal

            # State 3: Generate path to next exploration goal and execute it
            if self.state == State.GoToExplorationGoal:
                rospy.loginfo("Go to next goal")
                A = Planner(self.start_pose, next_pose, self.grid)
                path = A.run()
                if path is None:
                    self.state = State.GenerateExplorationGoal
                    continue
                self.cf.stop_hovering()
                self.path_executer.execute_path(path)
                self.state = State.RotateAndSearchForIntruder

            # State 4: Rotate 90 degrees and hover a while while waiting for intruder detection
            if self.state == State.RotateAndSearchForIntruder:
                rospy.loginfo("Checks for intruders")
                #self.cf.rotate(3, 5)
                self.cf.rotate()
                self.state = State.GenerateExplorationGoal
                self.cf.start_hovering()

            # State 5: Land on the ground when explorer can't find more space to explore
            if self.state == State.Landing:
                rospy.loginfo("Begins to land")
                self.cf.stop_hovering()
                self.cf.land()
                break

        rospy.loginfo("State machine finished!")

    def wait_for_pose(self):
        while not rospy.is_shutdown() and self.current_pose is None:
            continue

    def pose_callback(self, msg):
        self.current_pose = msg


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


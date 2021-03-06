#!/usr/bin/env python

import sys
import json
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
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
    """ State machine for mission planning and execution """

    def __init__(self, argv=sys.argv):
        rospy.loginfo("Initializing state machine")

        # Initialize map
        args = rospy.myargv(argv=argv)
        with open(args[1], 'rb') as f:
            world = json.load(f)

        self.height = 0.5  # Height at which to fly

        # Initialize class objects
        self.grid = GridMap(0.25, world, self.height)
        self.path_executer = PathExecution()
        self.tf = Transform()
        self.cf = Crazyflie("cf1")
        self.path_planner = Planner(self.grid)

        # Initialize subscriber
        self.sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, self.pose_callback)

        # Initialize exploration with starting pose
        self.current_pose = None
        self.wait_for_pose()
        self.start_pose = self.tf.transform2map(self.current_pose)
        self.explore = Explore(self.grid, self.start_pose)

        # Initialize state machine
        self.state = State.Init
        self.next_pose = None

        rospy.loginfo("state machine initialzed")
        self.states()

    def states(self):
        # Wait for localization to be initialized
        rospy.wait_for_message('is_initialized', Empty)

        rospy.loginfo("Taking off")
        while not rospy.is_shutdown():
            print("State:", self.state.name)

            # State 1: lift off
            if self.state == State.Init:
                self.cf.takeOff(self.height)
                self.state = State.RotateAndSearchForIntruder

            # State 2: Generate next exploration goal
            if self.state == State.GenerateExplorationGoal:
                self.wait_for_pose()
                rospy.loginfo("Generating the next exploration goal")
                self.start_pose = self.tf.transform2map(self.current_pose)
                self.next_pose = self.explore.next_goal(self.start_pose)
                if self.next_pose is None:
                    self.state = State.Landing
                else:
                    self.state = State.GoToExplorationGoal

            # State 3: Generate path to next exploration goal and execute it
            if self.state == State.GoToExplorationGoal:
                rospy.loginfo("Go to next goal")
                path = self.path_planner.run(self.start_pose, self.next_pose)
                if path is None:
                    self.state = State.GenerateExplorationGoal
                    continue
                self.cf.stop_hovering()
                self.path_executer.execute_path(path)
                self.state = State.RotateAndSearchForIntruder

            # State 4: Rotate and hover a while while waiting for intruder detection
            if self.state == State.RotateAndSearchForIntruder:
                rospy.loginfo("Checks for intruders")
                yaw = self.tf.quaternion2yaw(self.current_pose.pose.orientation)
                for r in range(3):
                    yaw += 90
                    # yaw = np.mod((yaw + 180), 360) - 180
                    self.cf.rotate(yaw)

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
        """ Wait for most recent pose to arrive """
        rospy.loginfo("Waiting for a valid pose!")
        while not rospy.is_shutdown() and self.current_pose is None:
            continue

    def pose_callback(self, msg):
        """ Retrieves the current pose of the drone in odometry frame """
        coordinate = self.tf.posestamped_to_array(self.tf.transform2map(msg))
        if self.grid.coordinate_in_bounds(coordinate):
            self.current_pose = msg
        # else:
        #     self.current_pose = None


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

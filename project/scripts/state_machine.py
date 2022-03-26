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
from std_msgs.msg import Bool, String
from grid_map import GridMap
from path_planner import Planner
from explore import Explore


def pose_callback(msg):
    global current_pose
    current_pose = msg


def localized_callback(msg):
    global localized
    localized = msg.data


def reached_goal_callback(msg):
    global reached_goal
    reached_goal = msg.data


class StateMachine(object):
    def __init__(self, argv=sys.argv):
        args = rospy.myargv(argv=argv)
        with open(args[1], 'rb') as f:
            world = json.load(f)

        self.grid = GridMap(0.2, world)
        self.explore = Explore(self.grid)

        # Subscribe to topics
        sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, pose_callback)
        sub_localize = rospy.Subscriber('/is_initialized', Bool, localized_callback)
        sub_path_executed = rospy.Subscriber('/reached_goal', Bool, localized_callback)

        # Instantiate publishers
        self.pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
        self.pub_motion = rospy.Publisher('/motion', String, queue_size=2)

        # Init state machine
        self.goal = None
        self.current_pose = None
        self.localized = False
        self.state = 0
        rospy.sleep(3)
        self.states()

    def states(self):
        next_pose = None
        # State 0: lift off and hover
        if self.state == 0:
            self.publish_motion("liftoff")
            self.state = 1
            rospy.sleep(1)

        while not rospy.is_shutdown() and self.state != 4:

            # State 1: Generate next exploration goal from explorer
            if self.state == 1:
                next_pose = self.explore.next_point()
                if next_pose is None:
                    self.state = 4
                else:
                    self.state = 2

            # State 2: Generate path to next exploration goal and execute it
            if self.state == 2:
                A = Planner(next_pose, self.grid)
                A.run()

                self.state = 3
                rospy.sleep(1)

            # State 3: Rotate 90 degrees and hover a while three times while waiting for intruder detection
            if self.state == 3:
                self.publish_motion("rotate")

                self.state = 1
                rospy.sleep(1)

        # State 4: Land on the ground when explorer can't find more space to explore
        if self.state == 4:
            self.publish_motion("land")
            rospy.sleep(1)

        # Error handling
        if self.state == 5:
            rospy.logerr("%s: State machine failed. Check your code and try again!")
            return

        rospy.loginfo("%s: State machine finished!")
        return

    def publish_motion(self, motion):
        msg = String()
        msg.data = motion
        self.pub_motion.publish(msg)


if __name__ == '__main__':
    rospy.init_node('state_machine')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

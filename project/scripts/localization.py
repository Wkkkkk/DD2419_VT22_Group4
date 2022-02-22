#!/usr/bin/env python2
import sys
import math
import json
import copy

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped, Quaternion
import tf2_geometry_msgs
from crazyflie_driver.msg import Position
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Bool
import numpy as np


def load_world(file_path):
    with open(file_path, 'rb') as f:
        world = json.load(f)
    markers = []
    for marker in world['markers']:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = marker['pose']['position']
        roll, pitch, yaw = marker['pose']['orientation']
        (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w) = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
        markers.append(p)

    return markers


def main(argv=sys.argv):
    rospy.init_node('localization')

    arg = rospy.myargv(argv=argv)
    
    world_json_file = "/home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/test.world.json"
    if len(arg) > 1:
        world_json_file = arg[1]
    
    markers = load_world(world_json_file)
    print(markers)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()

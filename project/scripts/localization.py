#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import PoseStamped, TransformStamped
from aruco_msgs.msg import MarkerArray


class localization(object):
    def __init__(self):
        pass

    def marker_callback(self, msg):
        pass


def main():
    rospy.init_node('localization')
    localization = Localization()
    aruco_subscriber = rospy.Subscriber('/aruco/markers', MarkerArray, localization.marker_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

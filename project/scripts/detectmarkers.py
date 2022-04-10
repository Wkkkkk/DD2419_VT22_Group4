#!/usr/bin/env python

import math
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3
from aruco_msgs.msg import MarkerArray


broadcaster = tf2_ros.TransformBroadcaster()

def marker_callback(marker_message):
    global broadcaster

    markers = marker_message.markers
    for marker in markers:
        t = TransformStamped()
        t.header = marker.header
        t.header.frame_id = 'cf1/camera_link'
        t.child_frame_id = '/aruco/detected{}'.format(marker.id)
        t.transform.translation = marker.pose.pose.position
        t.transform.rotation = marker.pose.pose.orientation

        broadcaster.sendTransform(t)

def main():
    rospy.init_node('detectmarkers')
    sub_aruco = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

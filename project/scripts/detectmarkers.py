#!/usr/bin/env python

import math
import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Vector3
from aruco_msgs.msg import MarkerArray


def marker_callback(marker_message):
    global broadcaster

    markers = marker_message.markers
    for marker in markers:
        t = TransformStamped()
        t.header = marker.header
        t.header.frame_id = 'cf1/camera_link'
        t.child_frame_id = '/aruco/detected{}'.format(marker.id)
        t.transform.translation.x = marker.pose.pose.position.x
        t.transform.translation.y = marker.pose.pose.position.y
        t.transform.translation.z = marker.pose.pose.position.z

        t.transform.rotation.x = marker.pose.pose.orientation.x
        t.transform.rotation.y = marker.pose.pose.orientation.y
        t.transform.rotation.z = marker.pose.pose.orientation.z
        t.transform.rotation.w = marker.pose.pose.orientation.w

        broadcaster.sendTransform(t)

def main():
    rospy.init_node('detectmarkers')
    broadcaster = tf2_ros.TransformBroadcaster()
    sub_aruco = rospy.Subscriber('/aruco/markers', MarkerArray, ch.marker_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

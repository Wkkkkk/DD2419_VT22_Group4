#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from aruco_msgs.msg import MarkerArray


def hcmatrix_from_transform(t):
    quats = [0, 0, 0, 0]
    quats[0] = t.transform.rotation.x
    quats[1] = t.transform.rotation.y
    quats[2] = t.transform.rotation.z
    quats[3] = t.transform.rotation.w

    hcm = quaternion_matrix(quats)

    hcm[0,3] = t.transform.translation.x
    hcm[1,3] = t.transform.translation.y
    hcm[2,3] = t.transform.translation.z

    return hcm


def transform_from_hcmatrix(hcm,header_frame,child_frame):
    quats = quaternion_from_matrix(hcm)

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header_frame
    t.child_frame_id = child_frame

    t.transform.translation.x = hcm[0,3]
    t.transform.translation.y = hcm[1,3]
    t.transform.translation.z = hcm[2,3]

    t.transform.rotation.x = quats[0]
    t.transform.rotation.y = quats[1]
    t.transform.rotation.z = quats[2]
    t.transform.rotation.w = quats[3]

    return t


class Localization(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.transform = TransformStamped()
        self.transform.header.frame_id = 'map'
        self.transform.child_frame_id  = 'cf1/odom'
        self.transform.transform.rotation.w = 1
        self.is_initialized = False

    def marker_callback(self, msg):
        markers = msg.markers
        self.is_initialized = True

        for marker in markers:
            marker_id = marker.id
            rospy.loginfo("Find marker: %s")

            # Find marker position in map frame
            m = self.tf_buffer.lookup_transform('map',
                                                'aruco/marker{}'.format(marker.id),
                                                rospy.Time(),
                                                rospy.Duration(5.0))
            # Find marker position in odom frame
            o = self.tf_buffer.lookup_transform('cf1/odom',
                                                'aruco/detected{}'.format(marker.id),
                                                rospy.Time(),
                                                rospy.Duration(5.0))

            M = hcmatrix_from_transform(m)
            O = hcmatrix_from_transform(o)
            O_inv = np.linalg.inv(O)

            # Calculate transform
            T = np.dot(M, O_inv)
            transform = transform_from_hcmatrix(T, 'map', 'cf1/odom')
            #print(transform)
            self.transform = transform


def main():
    rospy.init_node('localization')
    localization = Localization()
    aruco_subscriber = rospy.Subscriber('/aruco/markers', MarkerArray, localization.marker_callback)
    initialization_publisher = rospy.Publisher('is_initialized', Bool, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        localization.transform.header.stamp = rospy.Time.now()
        localization.broadcaster.sendTransform(localization.transform)

        initialization_publisher.publish(localization.is_initialized)
        rate.sleep()


if __name__ == "__main__":
    main()

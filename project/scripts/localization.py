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
        p.header.seq = marker['id']
        p.pose.position.x, p.pose.position.y, p.pose.position.z = marker['pose']['position']
        roll, pitch, yaw = marker['pose']['orientation']
        (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w) = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
        markers.append(p)

    return markers


def marker_detect_callback(msg):
    measurements = msg.markers
    for m in measurements:
        # find marker in camera frame
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'cf1/camera_link'
        pose.pose = m.pose.pose

        if not tf_buf.can_transform(pose.header.frame_id, 'map', pose.header.stamp, rospy.Duration(1)):
            rospy.logwarn_throttle(5.0, "No transform from %s to map" % pose.header.frame_id)
            return
        # tranform marker to map frame
        pose_map = tf_buf.transform(pose, 'map')

        # create transformedStamped to publish
        marker2map_trans = TransformStamped()
        marker2map_trans.header.stamp = m.header.stamp
        marker2map_trans.header.frame_id = 'map'
        marker2map_trans.child_frame_id  = 'aruco/detected' + str(m.id)
        marker2map_trans.transform.translation = pose_map.pose.position
        marker2map_trans.transform.rotation = pose_map.pose.orientation

        marker2map_broadcast.sendTransform(marker2map_trans)
        rospy.loginfo("transform marker %s to map" % m.id)

        # relocate
        update_map2odom_trans(m)


def update_map2odom_trans(measurement):
    global markers

    marker = next(m for m in markers if m.header.seq == measurement.id)
    print("find marker with id:", marker.header.seq)

    # calculate the difference
    diff = measurement.pose.pose - marker.pose

    # transform odom origin to map
    pose = PoseStamped()
    pose.header.stamp = measurement.header.stamp
    pose.header.frame_id = 'cf1/odom'
    pose.pose.position = Vector3()
    (pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w) = quaternion_from_euler(0,0,0)
    if not tf_buf.can_transform(pose.header.frame_id, 'map', pose.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % pose.header.frame_id)
        return
    odom = tf_buf.transform(pose, 'map')

    # update trans
    t.transform.translation.x = odom.pose.position.x + diff.pose.x
    t.transform.translation.y = odom.pose.position.y + diff.pose.y
    pass


rospy.init_node('localization')
is_initialized = False
markers = []

t = TransformStamped()
t.header.stamp = rospy.Time.now()
t.header.frame_id = 'map'
t.child_frame_id  = 'cf1/odom'
t.transform.translation = Vector3()
(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w) = quaternion_from_euler(0,0,0)


map2odom_broadcast = tf2_ros.TransformBroadcaster()
marker2map_broadcast = tf2_ros.TransformBroadcaster()
tf_buf    = tf2_ros.Buffer()
tf_lstn   = tf2_ros.TransformListener(tf_buf)

landmarker_subscriber  = rospy.Subscriber('/aruco/markers', MarkerArray, marker_detect_callback)
localization_publisher = rospy.Publisher('localization_initialized', Bool, queue_size=3)


def main(argv=sys.argv):
    arg = rospy.myargv(argv=argv)
    global markers
    world_json_file = "/home/maciejw/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/test.world.json"
    if len(arg) > 1:
        world_json_file = arg[1]

    markers = load_world(world_json_file)
    print(markers)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        map2odom_broadcast.sendTransform(t)

        rate.sleep()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3
from project.msg import Detection, DetectionArray
from aruco_msgs.msg import MarkerArray
import networkx as nx

# from detector_node_with_pose_est import categories
categories = {0: "no_bicycle", 1: "airport", 2: "dangerous_left",
                3: "dangerous_right", 4: "follow_left",
                5: "follow_right", 6: "junction", 7: "no_heavy_truck",
                8: "no_parking", 9: "no_stopping_and_parking",
                10: "residential", 11: "narrows_from_left",
                12: "narrows_from_right", 13: "roundabout", 14: "stop"}

def pose_from_transform(t):
    """Convert a C{geometry_msgs/TransformStamped} into Pose
 
    @param t: ROS message to be converted
    @return: Pose
    """
    p = Pose()
    p.orientation = t.transform.rotation
    p.position.x = t.transform.translation.x
    p.position.y = t.transform.translation.y
    p.position.z = t.transform.translation.z

    return p

def transform_from_marker(m, id):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'aruco/marker' + str(id)
    t.transform.translation = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t


def hcmatrix_from_transform(t):
    """Convert a C{geometry_msgs/TransformStamped} into 4*4 np arrays
 
    @param t: ROS message to be converted
    @return: transition matrix
    """
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
    """Convert a  4*4 np array into a C{geometry_msgs/TransformStamped} message
 
    @param hcm:  transition matrix
    @return: ROS message
    """
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


def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q


def transform_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return transform_to_pq(msg.transform)


class KalmanFilter:
    def __init__(self):
        self.is_initialized = False

        # initial state (location and velocity)
        self.mu = np.zeros((3, 1))  # [x, y, theta]
        # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
        self.P = np.eye(3)*0.1
        # state transition matrix: generalize the 2d version to 4d
        self.F = np.eye(3)
        # measurement matrix: reflect the fact that we observe x and y but not the two velocities
        self.H = np.eye(3)
        # measurement uncertainty
        self.R = np.eye(3) * 0.3
        self.R2 = np.eye(3) * 3
        # transition uncertainty
        self.Q = np.eye(3) * 0.01


    def update(self, msg, is_sign=True):
        p, q = transform_stamped_to_pq(msg)
        roll, pitch, yaw = euler_from_quaternion(q)

        # Measurement
        x_measured = p[0]
        y_measured = p[1]
        Z = np.array([[x_measured], [y_measured], [yaw]])
        Z[2] = (Z[2] + np.pi) % (2 * np.pi) - np.pi

        if not self.is_initialized:
            self.mu[0] = x_measured
            self.mu[1] = y_measured
            self.mu[2] = yaw
            self.is_initialized = True

        # Innovation
        y = Z - np.dot(self.H, self.mu)
        y[2] = (y[2] + np.pi) % (2 * np.pi) - np.pi

        # Kalman Gain
        R = self.R
        if is_sign:
            R = self.R2
        S = np.dot(np.dot(self.H, self.P), np.transpose(self.H)) + R
        K = np.dot(np.dot(self.P, np.transpose(self.H)), np.linalg.inv(S))

        # Posterier mu and sigma
        self.mu = self.mu + np.dot(K, y)
        self.P = np.dot((np.eye(3) - np.dot(K, self.H)), self.P)

        # Output 
        msg.transform.translation.x = self.mu[0]
        msg.transform.translation.y = self.mu[1]
        msg.transform.translation.z = 0

        (msg.transform.rotation.x,
         msg.transform.rotation.y,
         msg.transform.rotation.z,
         msg.transform.rotation.w) = quaternion_from_euler(0, 0, self.mu[2])  # yaw

        # predict
        self.mu = np.dot(self.F, self.mu)
        self.P = np.dot(np.dot(self.F, self.P), np.transpose(self.F)) + self.Q

        return msg


class Matcher:
    def __init__(self, static_marker_transforms):
        self.static_markers = {t.child_frame_id:t for t in static_marker_transforms}
        #print(self.static_markers)

    def match(self, transform_list):
        graph = nx.Graph()
        for transform in transform_list:
            frame_id = transform.child_frame_id
            graph.add_node(frame_id, bipartite=0)
            for marker_frame_id, marker_transform in self.static_markers.items():
                graph.add_node(marker_frame_id, bipartite=1)
                distance = self.calculate(transform, marker_transform)
                score = 1/distance
                if score is not None:
                    graph.add_edge(frame_id, marker_frame_id, weight=score)
        match_set = nx.max_weight_matching(graph)
        res = dict()
        for (node_1, node_2) in match_set:
            if node_1.startswith('aruco/marker'):
                node_1, node_2 = node_2, node_1
            res[node_1] = node_2
        return res


    def calculate(self, transform, static_transform):
        t1 = hcmatrix_from_transform(transform)
        t2 = hcmatrix_from_transform(static_transform)
        diff_abs = np.absolute(t1-t2)
        diff_sum = diff_abs.sum()
        #print("diff:", transform.child_frame_id, " ", static_transform.child_frame_id, " ", diff_sum)

        return diff_sum


class Localization(object):
    def __init__(self, matcher):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.initialization = rospy.Publisher('is_initialized', Empty, queue_size=1)

        self.transform = TransformStamped()
        self.transform.header.frame_id = 'map'
        self.transform.child_frame_id  = 'cf1/odom'
        self.transform.transform.rotation.w = 1

        self.kf = KalmanFilter()
        self.matcher = matcher

    def sign_callback(self, msg):
        detections = msg.detections
        for detection in detections:
            sign_id = detection.id
            if not sign_id in categories:
                print("Unknown id:", sign_id)
                continue

            sign_category = categories[sign_id]
            print("We have a sign:", sign_id, sign_category)
            if not self.tf_buffer.can_transform('map', 'world/roadsign_' + sign_category, rospy.Time(), rospy.Duration(3.0)):
                print("Can't find sign in the world", sign_category)
                continue

            # Find sign position in map frame
            m = self.tf_buffer.lookup_transform('map',
                                                'world/roadsign_' + sign_category,
                                                rospy.Time(),
                                                rospy.Duration(3.0))
            # Find sign position in odom frame
            o = self.tf_buffer.lookup_transform('cf1/odom',
                                                'detector/detectedsign_' + sign_category,
                                                rospy.Time(),
                                                rospy.Duration(3.0))
            M = hcmatrix_from_transform(m)
            O = hcmatrix_from_transform(o)
            O_inv = np.linalg.inv(O)

            # Calculate transform
            T = np.dot(M, O_inv)
            transform = transform_from_hcmatrix(T, 'map', 'cf1/odom')
            #print(transform)
            self.transform = self.kf.update(transform, True)
            print("Matched a sign:", sign_id, sign_category)

            # ready to take off
            self.initialization.publish(Empty())


    def marker_callback(self, msg):
        markers = msg.markers

        detected_markers = []
        for marker in markers:
            # Find detected marker position in map frame
            if self.tf_buffer.can_transform('map', 'aruco/detected{}'.format(marker.id), rospy.Time(), rospy.Duration(3.0)):
                t = self.tf_buffer.lookup_transform('map',
                                                    'aruco/detected{}'.format(marker.id),
                                                    rospy.Time(),
                                                    rospy.Duration(3.0))
                detected_markers.append(t)
                # p = pose_from_transform(t)
                # print("find marker:", marker.id, p)
        if not detected_markers:
            print("No match")
            return

        res = self.matcher.match(detected_markers)
        print("---")
        for detected, static in res.items():
            print("match:", detected, " with:", static)  

            # Find marker position in map frame
            m = self.tf_buffer.lookup_transform('map',
                                                static,
                                                rospy.Time(),
                                                rospy.Duration(3.0))
            # Find marker position in odom frame
            o = self.tf_buffer.lookup_transform('cf1/odom',
                                                detected,
                                                rospy.Time(),
                                                rospy.Duration(3.0))
            M = hcmatrix_from_transform(m)
            O = hcmatrix_from_transform(o)
            O_inv = np.linalg.inv(O)

            # Calculate transform
            T = np.dot(M, O_inv)
            transform = transform_from_hcmatrix(T, 'map', 'cf1/odom')
            #print(transform)
            self.transform = self.kf.update(transform)

            # ready to take off
            self.initialization.publish(Empty())



def main(argv=sys.argv):
    # Let ROS filter through the arguments
    args = rospy.myargv(argv=argv)

    print("Start localization")
    # Load world JSON
    with open(args[1], 'rb') as f:
        world = json.load(f)

    # Create a transform for each marker
    transforms = [transform_from_marker(m, index+1) for index, m in enumerate(world['markers'])]
    matcher = Matcher(transforms)

    rospy.init_node('localization')
    rospy.sleep(10)
    localization = Localization(matcher)
    aruco_subscriber = rospy.Subscriber('/aruco/markers', MarkerArray, localization.marker_callback)
    sign_subscriber  = rospy.Subscriber('/detected_sign', DetectionArray, localization.sign_callback)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        localization.transform.header.stamp = rospy.Time.now()
        localization.broadcaster.sendTransform(localization.transform)

        rate.sleep()


if __name__ == "__main__":
    print("?????")
    main()

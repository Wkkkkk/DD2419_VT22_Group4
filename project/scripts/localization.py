#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, TransformStamped
from aruco_msgs.msg import MarkerArray


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
        self.mu = np.zeros((6, 1))  # [x, y, theta, x', y', theta']

        # initial uncertainty: 0 for positions x and y, 1000 for the two velocities
        self.P = np.zeros((6,6))
        self.P[3, 3] = self.P[4, 4] = self.P[5, 5] = 1000
        # state transition matrix: generalize the 2d version to 4d
        self.F = np.eye(6)
        self.F[0,3] = self.F[1,4] = self.F[2,5] = 0.5
        # measurement matrix: reflect the fact that we observe x and y but not the two velocities
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1
        # measurement uncertainty: use 2x2 matrix with 0.1 as main diagonal
        self.R = np.eye(3) * 100
        self.Q = np.eye(6) * 0.01
        self.I = np.eye(6)


    def update(self, msg):
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
        S = np.dot(np.dot(self.H, self.P), np.transpose(self.H)) + self.R
        K = np.dot(np.dot(self.P, np.transpose(self.H)), np.linalg.inv(S))

        # Posterier mu and sigma
        self.mu = self.mu + np.dot(K, y)
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)

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


class Localization(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.initialization = rospy.Publisher('is_initialized', Empty, queue_size=1)

        self.transform = TransformStamped()
        self.transform.header.frame_id = 'map'
        self.transform.child_frame_id  = 'cf1/odom'
        self.transform.transform.rotation.w = 1

        self.kf = KalmanFilter()


    def data_association(self, marker):
        marker_id = None
        target_id = marker.id
        target_frame_id = 'aruco/detected{}'.format(target_id)
        
        n = 0
        min_distance = 100
        min_yaw = 100
        while True:
            match_frame_id = 'aruco/marker{}'.format(n)
            # Find transform between detected marker and all static markers
            try:
                trans = self.tf_buffer.lookup_transform(target_frame_id, match_frame_id, rospy.Time(), rospy.Duration(3.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                # print(target_frame_id, match_frame_id)
                # print(e)
                break  # if n exceeds number for last non-unique static marker in map, end the loop.
            p,q = transform_stamped_to_pq(trans)

            distance = np.linalg.norm(p)
            roll, pitch, yaw = euler_from_quaternion(q)
            #yaw = roll  # because of aruco marker orientation
            rospy.loginfo("Detect: %s matching %s distance %f", target_frame_id, match_frame_id, distance)
            if np.abs(yaw) <= math.pi / 6:
                if np.abs(yaw) <= min_yaw and distance <= min_distance:
                    rospy.loginfo("matched: %d", n)
                    marker_id = n
                    min_distance = distance
                    min_yaw = yaw
            n += 1
        return marker_id


    def marker_callback(self, msg):
        markers = msg.markers

        for marker in markers: 
            marker_id = self.data_association(marker)
            print("Find marker with id:", marker.id, " match it with:", marker_id)  
            if not marker_id:  # No matched marker
                continue
            else:
                rospy.loginfo("Find marker: %d", marker_id)

            # Find marker position in map frame
            m = self.tf_buffer.lookup_transform('map',
                                                'aruco/marker{}'.format(marker_id),
                                                rospy.Time(),
                                                rospy.Duration(3.0))
            # Find marker position in odom frame
            o = self.tf_buffer.lookup_transform('cf1/odom',
                                                'aruco/detected{}'.format(marker.id),
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


def main():
    rospy.init_node('localization')
    localization = Localization()
    aruco_subscriber = rospy.Subscriber('/aruco/markers', MarkerArray, localization.marker_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        localization.transform.header.stamp = rospy.Time.now()
        localization.broadcaster.sendTransform(localization.transform)

        rate.sleep()


if __name__ == "__main__":
    main()

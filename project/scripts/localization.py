#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler
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

        # Motion model
        self.A = np.eye(3)     # State transition matrix
        self.R = np.eye(3)     # Motion noise matrix

        # Measurement model
        self.C = np.ones(3)    # Observation matrix. #array([[1,1,1]])
        self.Q = np.eye(3)*0.1 # Observation noise matrix

        self.mu = None
        self.sigma = None
        self.z = None


    def update(self, msg):
        p, q = transform_stamped_to_pq(msg)
        roll, pitch, yaw = euler_from_quaternion(q)

        # Measurement
        self.z = np.array([p[0], p[1], yaw])  # x, y, yaw

        if not self.is_initialized:
            self.mu = self.z
            self.sigma = np.eye(3)
            self.is_initialized = True

        # Estimated belief
        self.mu, self.sigma = self.kf_predict()

        # Posterior distribution and Kalman Gain
        self.mu, self.sigma, K = self.kf_update()

        # Output 
        msg.transform.translation.x = self.mu[0]
        msg.transform.translation.y = self.mu[1]

        (msg.transform.rotation.x,
         msg.transform.rotation.y,
         msg.transform.rotation.z,
         msg.transform.rotation.w) = quaternion_from_euler(0, 0, self.mu[2])  # yaw


        return msg


    def kf_update(self):
        # Kalman Gain
        K_term = np.dot(self.C.T, np.linalg.inv((np.dot(self.C, np.dot(self.sigma, self.C.T)) + self.Q)))
        K = np.dot(self.sigma, K_term)
        # Innovation
        eta = self.z - np.dot(self.C, self.mu)

        #print(self.z.shape, K.shape, eta.shape)
        # Compute posterior
        mu = self.mu + np.dot(K, eta) # Mean of the state
        I = np.eye(3)
        sigma = np.dot((I - np.dot(K, self.C)), self.sigma) # Covariance of the state

        return mu, sigma, K

    def kf_predict(self):
        # Estimated mean of the state (no control)
        mu = np.dot(self.A, self.mu)

        # Estimated covariance of the state
        sigma = np.dot(self.A, np.dot(self.sigma, self.A.T)) + self.R

        return mu, sigma


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

        self.kf = KalmanFilter()


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
            self.transform = self.kf.update(transform)


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

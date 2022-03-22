#!/usr/bin/env python

<<<<<<< HEAD
from socket import timeout
from sqlite3 import Timestamp
from syslog import LOG_INFO
=======
from sqlite3 import Timestamp
>>>>>>> 31c83e87f7356d6990a4f5836bf7eff5002a59ae
from numpy import broadcast
import rospy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
<<<<<<< HEAD
from std_msgs.msg import Int32
#from msg import Detected

categories = {0:"no_bicycle", 1:"airport" , 2: "dangerous_left", 3:"dangerous_right", 4: "follow left",
=======
from std_msgs import Int32
#from msg import Detected

categories = {0:"no bicycle", 1:"airport" , 2: "dangerous left", 3:"dangerous right", 4: "follow left",
>>>>>>> 31c83e87f7356d6990a4f5836bf7eff5002a59ae
               5:"follow right", 6:"junction", 7:"no heavy truck", 8:"no parking", 9:"no stopping and parking",
               10:"residential", 11:"narrows from left", 12:"narrows from right", 13:"roundabout", 14:"stop"}

def sign_callback(msg):

    # Here we should have a pose estimation based on the detected bounding box
    # For the moment we just get the static transform from map to sign and post this as the current pose
    # In the future we will have a estimated pose

<<<<<<< HEAD
    #time_stamp = msg.header.stamp
    time_stamp = rospy.Time.now()
    sign_id = msg.data
    #print(categories[sign_id])

    if tf_buf.can_transform('map', ('world/roadsign' + categories[sign_id]), time_stamp, timeout= rospy.Duration(0.2)):
        trans = tf_buf.lookup_transform(  'map',('world/roadsign' + categories[sign_id]), time_stamp, timeout=rospy.Duration(0.2))
        #print(categories[sign_id])
    else:
        return
    sign_pose_transform = TransformStamped()
    print(trans)
    sign_pose_transform.header.stamp = time_stamp
    sign_pose_transform.header.frame_id = 'map'
    sign_pose_transform.child_frame_id = 'detector/detectedsign_' + categories[sign_id]
    sign_pose_transform.transform.translation = trans.transform.translation

    sign_pose_transform.transform.rotation = trans.transform.rotation


    print("broadcasting: " + categories[sign_id])
    broadcaster.sendTransform(sign_pose_transform)
=======
    time_stamp = msg.header.stamp
    sign_id = msg.data

    trans = self._tf_buf.lookup_transform('world/roadsign_' + categories[sign_id]), '/map', timestamp, timeout=Duration(seconds=1.0))

    sign_pose_transform = TransformStamped()

    sign_pose_transform.header.stamp = time_stamp
    sign_pose_transform.header.frame_id = 'map'
    sign_pose_transform.child_frame_id = 'detector/detectedsign_' + categories[sign_id]
    sign_pose_transform.transform.translation = trans.pose.position
    sign_pose_transform.transform.rotation = trans.pose.orientation

    broadcaster.sendTransform(trans)
>>>>>>> 31c83e87f7356d6990a4f5836bf7eff5002a59ae

def transform_to_map(sign_in_camera):

    # Convert markers type to pose stamped for transformation
    sign_pose_camera = PoseStamped()
    sign_pose_camera.pose = sign_in_camera.pose.pose
    sign_pose_camera.header.frame_id = 'cf1/camera_link' #m.header.frame_id
    sign_pose_camera.header.stamp = sign_in_camera.header.stamp # rospy.Time(0)
    sign_pose_map = 0
    timeout = rospy.Duration(0.5)

    # Check if there is a transform
    if not tf_buf.can_transform(sign_pose_camera.header.frame_id, 'map', sign_pose_camera.header.stamp,timeout):
        rospy.logwarn_throttle(5.0, 'No transform from %s to map' % sign_pose_camera.header.frame_id)
        return sign_pose_map, sign_in_camera.id

    # Do the transform from camera_link to map frame
    sign_pose_map = tf_buf.transform(sign_pose_camera, 'map')

    return sign_pose_map, sign_in_camera.id

def create_transform(sign_in_map, sign_id):

    # Create Transform message
    transform_sign_map = TransformStamped()
    transform_sign_map.header.stamp = sign_in_map.header.stamp
    transform_sign_map.header.frame_id = 'map'
    transform_sign_map.child_frame_id = 'detector/detectedsign' + str(sign_id)
    transform_sign_map.transform.translation = sign_in_map.pose.position
    transform_sign_map.transform.rotation = sign_in_map.pose.orientation

    return transform_sign_map

rospy.init_node('sign_position')

tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()
#rospy.sleep(5)

def main():
    print("running...")
    # Initiates listener to the sign topic then spins the node, WE NEED TOPIC NAME AND MSG TYPE HERE
    sign_listener = rospy.Subscriber('/detected_sign', Int32, sign_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

from numpy import broadcast
import rospy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
import rospy
import tf2_ros 
from geometry_msgs.msg import TransformStamped

def sign_callback(msg):

    # For each currently detected sign (TYPE OF MESSAGE?)
    for sig in msg.markers:
        
        # Transform marker from camera to map
        sign_in_map, sig_id = transform_to_map(sig)
        
        if sign_in_map != 0:
            # Create transform
            transform = create_transform(sign_in_map, sig_id)
            # Publish transform    
            broadcaster.sendTransform(transform)

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

rospy.init_node('detected_sign_transform')

tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()
#rospy.sleep(5)

def main():
    print("running...")
    # Initiates listener to the sign topic then spins the node, WE NEED TOPIC NAME AND MSG TYPE HERE
    sign_listener = rospy.Subscriber('/sign_pose', PoseStamped, sign_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
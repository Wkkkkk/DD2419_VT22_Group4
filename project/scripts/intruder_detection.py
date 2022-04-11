#!/usr/bin/env python

import string
import rospy
import os
import numpy as np
import random
import sys
import json

from std_msgs.msg import Int32, String
from project.msg import Detection, DetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class intruder_detection:

    def __init__(self, argv=sys.argv):


          # world_path = '/path'

          # Let ROS filter through the arguments
          args = rospy.myargv(argv=argv)

          # Load world JSON
          with open(args[1], 'rb') as f:
              self.world = json.load(f)

          # f = open(world_path)

          # self.world = json.load(f)

          self.categories = {0:"no_bicycle", 1:"airport" , 2: "dangerous_left", 3:"dangerous_right", 4: "follow_left",
                         5:"follow_right", 6:"junction", 7:"no_heavy_truck", 8:"no_parking", 9:"no_stopping_and_parking",
                         10:"residential", 11:"narrows_from_left", 12:"narrows_from_right", 13:"roundabout", 14:"stop"}

          self.signs_in_world = self.signs_in_world()

          # Init TF
          self.tf_buf   = tf2_ros.Buffer()
          self.tf_listner  = tf2_ros.TransformListener(self.tf_buf)
          self.broadcaster = tf2_ros.StaticTransformBroadcaster()

          rospy.sleep(1)

          self.detected_sub = rospy.Subscriber("/detected_sign", DetectionArray, self.callback, queue_size = 1)
          self.intruder_pub = rospy.Publisher("/intruder_topic", String, queue_size = 2)
          # self.intruder_pose_pub = rospy.Publisher("/intruder_topic", String, queue_size = 10)



          rospy.loginfo('Intruder detection running')

    def callback(self, msg):
        for m in msg.detections:
            detected_sign_id = m.id

            #if m.confidence >= 0.9:

            if detected_sign_id in self.signs_in_world:
                m.intruder = False
                sign = self.categories[detected_sign_id]
                self.intruder_pub.publish("NO INTRUDER DETECTED: We have detected " + sign)
                rospy.loginfo("NO INTRUDER DETECTED: We have detected " + sign)

            else:
                m.intruder = True
                sign = self.categories[detected_sign_id]
                self.intruder_pub.publish("INTRUDER DETECTED: We have detected " + sign)
                rospy.loginfo("INTRUDER DETECTED: We have detected " + sign)


                   #   # marker pose is in frame camera_link
                if not self.tf_buf.can_transform('map', 'cf1/camera_link', m.header.stamp, rospy.Duration(1)):
                   rospy.logwarn('pose_estimation: No transform from %s to map', 'cf1/camera_link')
                   print("heeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeej ingen transform :(")
                   return

                sign_transform = self.tf_buf.transform(m.pose, 'map')
                print('hit')
                t = TransformStamped()

                t.header.stamp = m.header.stamp

                t.header.frame_id = 'map'
                # trans.header.frame_id = 'map'
                t.child_frame_id = 'detector/detectedIntruder_' + sign

                t.transform.translation.x = sign_transform.pose.position.x
                t.transform.translation.y = sign_transform.pose.position.y
                t.transform.translation.z = sign_transform.pose.position.z
                t.transform.rotation.x = sign_transform.pose.orientation.x
                t.transform.rotation.y = sign_transform.pose.orientation.y
                t.transform.rotation.z = sign_transform.pose.orientation.z
                t.transform.rotation.w = sign_transform.pose.orientation.w

                self.broadcaster.sendTransform(t)

            #else:
             #   return

    def signs_in_world(self):

        signs_in_world = {}

        sign_value = [sign['sign'] for sign in self.world["roadsigns"]]

        val_list = list(self.categories.values())

        sign_ID = [val_list.index(sign_id) for sign_id in sign_value]

        for i in range(len(sign_ID)):
            signs_in_world[sign_ID[i]] = sign_value[i]

        return signs_in_world

if __name__ == '__main__':
    rospy.init_node('Intruder_detection')
    try:
        intruder_detection()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

#!/usr/bin/env python

import string
import rospy
from std_msgs.msg import Int32, String
import os
import numpy as np
import random
import sys
from project_perception.msg import Detection, DetectionArray

class intruder_detection:

    def __init__(self, argv=sys.argv):

          rospy.loginfo('Intruder detection running')

          # Let ROS filter through the arguments
          args = rospy.myargv(argv=argv)

          # Load world JSON
          with open(args[1], 'rb') as f:
              self.world = json.load(f)

          self.categories = {0:"no bicycle", 1:"airport" , 2: "dangerous left", 3:"dangerous right", 4: "follow left",
                         5:"follow right", 6:"junction", 7:"no heavy truck", 8:"no parking", 9:"no stopping and parking",
                         10:"residential", 11:"narrows from left", 12:"narrows from right", 13:"roundabout", 14:"stop"}

          self.signs_in_world = self.signs_in_world()

          self.detected_sub = rospy.Subscriber("/detected_sign", DetectionArray, self.callback)

          self.intruder_pub = rospy.Publisher("/intruder_topic", String, queue_size = 10)
          # self.intruder_pose_pub = rospy.Publisher("/intruder_topic", String, queue_size = 10)

    def callback(self, msg):

        for m in msg:

            detected_sign_id = m.classification

            if detected_sign_id in self.signs_in_world:
                m.intruder = False
                sign = self.categories[detected_sign_id]
                self.intruder_pub.publish("NO INTRUDER DETECTED: We have detected " + sign)

            else:
                m.intruder = True
                sign = self.categories[detected_sign_id]
                self.intruder_pub.publish("INTRUDER DETECTED: We have detected " + sign)


    def signs_in_world(self):

        signs_in_world = {}

        sign_value = [sign for sign in self.world["roadsigns"]["sign"]]

        val_list = list(self.categories.values())

        sign_ID = [value_list.index(sign_id) for sign_id in sign_value]

        for i in range(len(sign_ID)):
            signs_in_world[sign_ID[i]] = sign_value[i]

        return signs_in_world

if __name__ == '__main__':
    rospy.init_node('Intruder_detection')
    try:
        StateMachine()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

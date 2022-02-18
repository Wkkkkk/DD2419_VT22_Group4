#!/usr/bin/env python

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/bbox", Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)

  def callback(self,data):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Extract the dimension of the image
    (rows,cols,channels) = cv_image.shape

    # Here we want the detector to run, or it runns in the background and publishes info to some topic
    """DETECTOR"""
    
    box = True

    # Here we want to extract parameters x,y,width,height for the bounding box
    x = 100
    y = 100
    width = 100
    height = 100
    

    #Set start (top left corner), end (bottom right corner), box colour, box thickness
    start = (x,y)
    end = (x+width,y+height)
    box_colour = (255,0,0)
    thickness = 2

    # Set text parameters for label
    id = "Sign ID"
    org = (x, y-10)     # Position of texts lower left corner
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.6
    text_colour = (255, 0, 0)
    text_thickness = 1

    # If we have a box, paint it in the image with a text label
    if box:
      cv2.rectangle(cv_image, start, end, box_colour, thickness)
      cv2.putText(cv_image, id, org, font, fontScale, text_colour, text_thickness)

    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('bbox', anonymous=True)

  ic = image_converter()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


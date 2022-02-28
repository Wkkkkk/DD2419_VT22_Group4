#!/usr/bin/env python

# An initial attempt for the detection node, I think it's better to convert it to a class (like in flight camp).
import string
from PIL import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as Img
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import PoseStamped
import utils
import torch
from std_msgs import Int32
#from detector import Detector
import os
# from msg import Detected

dir = os.path.abspath(os.getcwd())

categories = {0:"no bicycle", 1:"airport" , 2: "dangerous left", 3:"dangerous right", 4: "follow left",
               5:"follow right", 6:"junction", 7:"no heavy truck", 8:"no parking", 9:"no stopping and parking",
               10:"residential", 11:"narrows from left", 12:"narrows from right", 13:"roundabout", 14:"stop"}

# here we should loaf in a pose file with all the known poses for the signs so we can publish them staticaly
# pose_file = {0:}

def callback(Image):
   global bridge

   time_stamp = Image.header.stamp

   # convert ros image to cv2
   try:
      cv_image = bridge.imgmsg_to_cv2(Image, "bgr8")
   except CvBridgeError as e:
      print(e)

   # convert to rgb
   RGB_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

   # convert to pillow image
   PIL_image  = Img.fromarray(RGB_image)

   # What follows here is just some weird structuring of the data I had to do for it to fit into the demands of the model, not sure why.
   # But the list detect_images is what is passed to the model.
   detect_images = []
   torch_image, _ = detector.input_transform(PIL_image, [])
   detect_images.append(torch_image)

   if detect_images:
      detect_images = torch.stack(detect_images)
      detect_images = detect_images.to(device)

   # We run the detector/model/network on the image here bbs is the decoded data
   #   bbs = detector(detect_images)
   with torch.no_grad():
      out = detector(detect_images)
      bbs = detector.decode_output(out, 0.5)

      # Uncomment this part to test if it publishes the tranform for detected sign
      """publish_detection(bbs)"""

      bounding_box(bbs, cv_image)
   # This will eventually post the pose, static for now
   # sign_pose(bbs)

   # Here we run the function that takes the bbox parameters and print rectangles and labels in the image

# This function publishes the detected values to the pose estimator
def publish_detection(bbs, time_stamp):
   if len(bbs[0]) != 0:
      bb = max(bbs[0], key=lambda j:j["confidence"])

      # detected_sign_param = Detected()
      # detected_sign_param.header.stamp = time_stamp
      # detected_sign_param.x = float(bb['x'])
      # detected_sign_param.y = float(bb['y'])
      # detected_sign_param.width = float(bb['width'])
      # detected_sign_param.height =float(bb['height'])
      # detected_sign_param.classification = bb['category']

      detected_sign_param = bb['category']

      detected_pub.publish(detected_sign_param)


# function to print bboxes
def bounding_box(detections, cv_image):
   box_colour = (255,0,0)
   thickness = 2
   font = cv2.FONT_HERSHEY_SIMPLEX
   fontScale = 0.6
   text_colour = (255, 0, 0)
   text_thickness = 1

   # The values are tensors in the decoded data structure so i convert to float, not sure if neccesary
   # structure of detections is list[list[dict]]

   # Since bbs CAN be a list for detections of multiple images we run a for loop here but it likely only runs once since we process one
   # image at a time.
#   for i, bbs in enumerate(detections):
   # Pick box with highest confidance
   if len(detections[0]) != 0:
      bb = max(detections[0], key=lambda j:j["confidence"])
      x = int(bb['x'].item())
      y = int(bb['y'].item())
      width = int(bb['width'].item())
      height = int(bb['height'].item())

      classification = categories[bb['category']]

      id = classification
      # Start is top left corner and end is bottom right
      start = (x,y)
      end = (x+width,y+height)

      #This is the point that anchors the text label in the image
      org = (x, y-10)

      cv2.rectangle(cv_image, start, end, box_colour, thickness)
      cv2.putText(cv_image, id, org, font, fontScale, text_colour, text_thickness)

   # Publish the image
   try:
      image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
   except CvBridgeError as e:
      print(e)

# This should post the pose, Not complete yet
"""
def sign_pose(detections):
   for i in detections:
      for bb in i:
         sign_id = bb['category']
         # for milestone 1 we can have fixed poses for the signs on file on file
         pose = PoseStamped(None,0,0,0,0,0,0) # pose_file[sign_id]
         pose_pub.publish(pose)
"""


# Function that runns the detector
#def detector(detect_image):
#   with torch.no_grad():
#      out = model(detect_image)
#      bbs = model.decode_output(out, 0.5)
#   return bbs

# Init node
rospy.init_node('detect_node')

# Init publisher
image_pub = rospy.Publisher("/bbox", Image, queue_size=10)
detected_pub = rospy.Publisher("/detected_sign", Int32, queue_size=2)

# Init detector with trained weights and set to evaluation mode
device = torch.device('cuda')
detector = Detector().to(device)
#dataloader = utils.load_model(detector, "/home/miguelclemente/dd2419_ws/src/part3/scripts/det_2022-02-20_19-48-10-524174.pt" , device)
path = dir + "/scripts/det_2022-02-20_19-48-10-524174.pt"
dataloader = utils.load_model(detector, path, device)

detector.eval()
bridge = CvBridge()



def main():


   image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, callback)

   rospy.spin()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

# An initial attempt for the detection node, I think it's better to convert it to a class (like in flight camp).
from ast import Global
import string
from PIL import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as Img
from sensor_msgs.msg import Image, CameraInfo
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import utils
import torch
from std_msgs.msg import Int32
from detector import Detector
import os
import numpy as np
# from msg import Detected

dir = os.path.abspath(os.getcwd())

categories = {0:"no bicycle", 1:"airport" , 2: "dangerous left", 3:"dangerous right", 4: "follow left",
               5:"follow right", 6:"junction", 7:"no heavy truck", 8:"no parking", 9:"no stopping and parking",
               10:"residential", 11:"narrows from left", 12:"narrows from right", 13:"roundabout", 14:"stop"}

def callback(Image):
   global bridge

   Image_header = Image.header

   # convert ros image to cv2
   try:
      cv_image = bridge.imgmsg_to_cv2(Image, "bgr8")
   except CvBridgeError as e:
      print(e)

   # convert to rgb
   RGB_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

   # convert to pillow image
   PIL_image  = Img.fromarray(RGB_image)

   #REMOVE DISTORTION! WIP
   h,w = PIL_image.shape[:2]
   new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_parameters, (w,h), 1, (w,h))
   dst = cv2.undistort(PIL_image, camera_matrix, distortion_parameters, None, new_camera_matrix)
   x, y, w, h = roi
   dst = dst[y:y+h, x:x+w]

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
      publish_detection(bbs, timestamp)

      bounding_box(bbs, cv_image)

# This function publishes the detected values to the pose estimator
# def publish_detection(bbs, time_stamp):
#    if len(bbs[0]) != 0:
#       bb = max(bbs[0], key=lambda j:j["confidence"])
#
#       # detected_sign_param = Detected()
#       # detected_sign_param.header.stamp = time_stamp
#       # detected_sign_param.x = float(bb['x'])
#       # detected_sign_param.y = float(bb['y'])
#       # detected_sign_param.width = float(bb['width'])
#       # detected_sign_param.height =float(bb['height'])
#       # detected_sign_param.classification = bb['category']
#
#       detected_sign_param = bb['category']
#
#       detected_pub.publish(detected_sign_param)


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

      bb_info = {'x': x, 'y': y, 'width': width, 'height': height, 'cat': classification}
      id = classification
      # Start is top left corner and end is bottom right
      start = (x,y)
      end = (x+width,y+height)

      #This is the point that anchors the text label in the image
      org = (x, y-10)

      cv2.rectangle(cv_image, start, end, box_colour, thickness)
      cv2.putText(cv_image, id, org, font, fontScale, text_colour, text_thickness)

      pose_estimation(cv_image, bb_info)

   # Publish the image
   try:
      image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
   except CvBridgeError as e:
      print(e)

def keypoints_for_estimation(matches, kp_object, kp_image):
    keypoints_3d = []
    keypoints_2d = []

    for i in range(10):
        object_index = matches[i].trainIdx
        image_index = matches[i].queryIdx
        print(image_index)
        keypoints_3d.append(kp_object[object_index])
        keypoints_2d.append(kp_image[image_index])

    return keypoints_3d, keypoints_2d

def object_keypoint_to_3d(object_keypoints):
    object_points = []
    for i in object_keypoints:
        object_points.append((i.pt[0], i.pt[1], 0))
    return np.array(object_points)

def image_keypoint_to_2d(image_keypoints):
    image_points = []
    for i in image_keypoints:
        image_points.append((i.pt[0], i.pt[1],))
    return np.array(image_points)

def pose_estimation(camera_image, bb_info):
    global Image_header

    # convert camera image to gray scale
    cv_gray = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)

    # Bounding box size
    x = bb_info['x']
    y = bb_info['y']
    width = bb_info['width']
    height = bb_info['height']

    # Cropp image to only include bounding box info
    cropped_img = cv_gray[y:y+width, x:x+height]

    # Import the cannonical traffic sign based on detected class
    base_img = cv2.imread("traffic_signs/" + bb_info['cat']  + ".jpg", cv2.IMREAD_COLOR)

    # convert cannonical image to gray scale
    base_gray = cv2.cvtColor(base_img, cv2.COLOR_BGR2GRAY)

    # Initiate SIFT detector
    sift = cv2.SIFT_create()

    # find the keypoints and descriptors with SIFT for both images
    kp1, des1 = sift.detectAndCompute(base_gray, None)
    kp2, des2 = sift.detectAndCompute(cropped_img, None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

    # Match descriptors with brute force
    matches = bf.match(des1, des2)

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance

    # Find the best matching keypoints from the matches
    keypoints_3d, keypoints_2d = keypoints_for_estimation(matches, kp1, kp2)

    # Convert keypoints into 3d object points
    object_points = object_keypoint_to_3d(keypoints_3d)

    # Convert keypoints into 2d image points
    image_points = image_keypoint_to_2d(keypoints_2d)

    # Find rotation and translation vectors for pose estimation
    retval, rvec, tvec = cv2.solvePnP(object_points, image_points, mtx, dist)

    sign_pose = PoseStamped()

    sign_pose.header = Image_header

    sign_pose.position.x = tvec[0]
    sign_pose.position.y = tvec[1]
    sign_pose.position.z = tvec[2]

    publish_pose(sign_pose, bb_info[cat])

    # HERE WE SHOULD ESTIMATE rotation

# Does the transform and publishes the map to detected sign transfomr
def publish_pose(sign_pose, category):
    broadcaster = tf2_ros.TransformBroadcaster()

    trans = TransformStamped()

    trans.header = sign_pose.Image_header
    trans.header.frame_id = 'map'
    trans.child_frame_id = 'detector/detectedsign_' + category

        # marker pose is in frame camera_link
    if not tf_buf.can_transform('map', sign_pose.header.frame_id, sign_pose.header.stamp, rospy.Duration(1)):
        rospy.logwarn('pose_estimation: No transform from %s to map', sign_pose.header.frame_id)
        return

    sign_transform = tf_buf.transform(sign_pose, 'map')

    trans.transform.translation.x = sign_transform.position.x
    trans.transform.translation.y = sign_transform.position.y
    trans.transform.translation.z = sign_transform.position.z

    broadcaster.sendTransform(trans)

# Init node
rospy.init_node('detect_node')

# Init publisher
image_pub = rospy.Publisher("/bbox", Image, queue_size=10)
detected_pub = rospy.Publisher("/detected_sign", Int32, queue_size=2)

# Init TF
tf_buf   = tf2_ros.Buffer()
tf_listner  = tf2_ros.TransformListener(tf_buf)

# Init detector with trained weights and set to evaluation mode
device = torch.device('cuda')
detector = Detector().to(device)
#dataloader = utils.load_model(detector, "/home/miguelclemente/dd2419_ws/src/part3/scripts/det_2022-02-20_19-48-10-524174.pt" , device)
path = dir + "/scripts/det_2022-02-20_19-48-10-524174.pt"
dataloader = utils.load_model(detector, path, device)

detector.eval()
bridge = CvBridge()


def caminfo(caminfo_msg):
   global camera_matrix, distortion_parameters

   camera_matrix = caminfo_msg.K
   distortion_parameters = caminfo_msg.D

def main():
   image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, callback)
   rospy.spin()


if __name__ == '__main__':
   caminfo_sub = rospy.Subscriber("/cf1/camera/camera_info", CameraInfo, caminfo)

   main()

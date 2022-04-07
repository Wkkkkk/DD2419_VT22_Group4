#!/usr/bin/env python

from ast import Global
import string
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

class sign_detection:

    def __init__(self):

        dir = dir = os.path.abspath(os.getcwd())

        self.tvec = [0,0,0]
        self.rvec = [0,0,0]
        self.current_id = -1

        self.mtx = np.array([[207.89466123, 0.,327.20867249],
                        [0., 205.19301437, 242.26158286],
                        [0., 0., 1.]])

        # Distortion matrix
        self.dist = np.array([ 0.17486646, -0.1696708,  -0.00025329,  0.00033005,  0.03769554])

        self.expand = 10
        # Categoriy name
        self.categories = {0:"no bicycle", 1:"airport" , 2: "dangerous left", 3:"dangerous right", 4: "follow left",
                       5:"follow right", 6:"junction", 7:"no heavy truck", 8:"no parking", 9:"no stopping and parking",
                       10:"residential", 11:"narrows from left", 12:"narrows from right", 13:"roundabout", 14:"stop"}

        # Names for cannonical images
        self.file_name = {0:"no_bicycle", 1:"airport" , 2: "dangerous_left", 3:"dangerous_right", 4: "follow_left",
                       5:"follow_right", 6:"junction", 7:"no_heavy_truck", 8:"no_parking", 9:"no stopping_and_parking",
                       10:"residential", 11:"narrows_from_left", 12:"narrows_from_right", 13:"roundabout", 14:"stop"}

        # Reference keypoint and descriptors
        self.ref = self.reference_features()

        # Init detector with trained weights and set to evaluation mode
        device = torch.device('cuda')
        self.detector = Detector().to(device)

        path = dir + "/scripts/det_2022-02-20_19-48-10-524174.pt"
        dataloader = utils.load_model(detector, path, device)
        detector.eval()

        # Init cv bridge
        self.bridge = CvBridge()

        # Init publisher
        self.image_pub = rospy.Publisher("/bbox", Image, queue_size=10)
        self.detected_sign_pub = rospy.Publisher("/detected_sign_pose", PoseStamped, queue_size=2)
        self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback)

        # Init TF
        self.tf_buf   = tf2_ros.Buffer()
        self.tf_listner  = tf2_ros.TransformListener(self.tf_buf)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Confidence threshold for detecting images.
        self.confidence = 0.9

    # Calculates features in cannonical images for later
    def reference_features(self):

        # Initiate SIFT detector
        sift = cv2.xfeatures2d.SIFT_create()

        # structure {name1: {kp1:(), des1:()}, name2: {kp2:(), des2:()}}
        ref = {}

        # import images
        for i in range(len(self.file_name)
            sign_file_name = self.file_name[i]
            sign_name = self.categories[i]

            base_img = cv2.imread("traffic_signs/" + sign_file_name  + ".jpg", cv2.IMREAD_COLOR)

            # convert cannonical image to gray scale
            base_gray = cv2.cvtColor(base_img, cv2.COLOR_BGR2GRAY)

            base_gray = cv2.resize(base_gray, (0,0), fx=0.1, fy=0.1)

            b_height, b_width = base_gray.shape

            kp, des = sift.detectAndCompute(base_gray, None)

            ref[sign_name] = {'kp': kp, 'des': des, 'width': b_width, 'height': b_height}

        return ref

    # Calculated and publishes the bpounding box image, returns bb_info dict
    def bounding_box(self, detections, cv_image):
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
       # for i, bbs in enumerate(detections):
       # Pick box with highest confidance
       if len(detections[0]) != 0:
          bb = max(detections[0], key=lambda j:j["confidence"])
          x = int(bb['x'].item())
          y = int(bb['y'].item())
          width = int(bb['width'].item())
          height = int(bb['height'].item())

          classification = self.categories[bb['category']]

          self.current_id = classification

          bb_info = {'x': x, 'y': y, 'width': width, 'height': height, 'cat': classification}

          # id = classification
          # Start is top left corner and end is bottom right
          start = (x,y)
          end = (x+width,y+height)

          #This is the point that anchors the text label in the image
          org = (x, y-10)

          cv2.rectangle(cv_image, start, end, box_colour, thickness)
          cv2.putText(cv_image, classification, org, font, fontScale, text_colour, text_thickness)

          return bb_info


       # Publish the image
       try:
          image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
       except CvBridgeError as e:
          print(e)

    # Extracts features and descriptors from camera image, returns keypoints and descriptors
    def extract_features(self, camera_image, bb_info):

        expand = self.expand
        # convert camera image to gray scale
        cv_gray = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)

        # Bounding box size
        x = bb_info['x']
        y = bb_info['y']
        width = bb_info['width']
        height = bb_info['height']
        sign_name = bb_info['cat']

        start = (x,y)
        end = (x+width,y+height)

        # Cropp image to only include bounding box info
        # cropped_img = cv_gray[y:y+width, x:x+height]

        w, h = cv_gray.shape[:2]
        mask = np.zeros([w,h], dtype=np.uint8)
        mask[start[1]-expand:end[1]+expand,start[0]-expand:end[0]+expand]=1

        cropped_img = cv2.bitwise_and(cv_gray,cv_gray,mask = mask)

        sift = cv2.xfeatures2d.SIFT_create()

        kp_camera, des_camera = sift.detectAndCompute(cropped_img, None)

        return kp_camera, des_camera

    # Matches the features between cannonical image and camera image, returns object and image image_points
    # for Pose estimation
    def match_features(self, keypoints, descriptor):

        sign_name = bb_info['cat']

        keypoint_object = self.ref[sign_name]['kp']
        descriptor_object = self.ref[sign_name]['des']
        image_width = self.ref[sign_name]['width']
        image_height = self.ref[sign_name]['height']

        # Check if we detect a new sign
        if self.current_id != bb_info['cat']:
            self.tvec[0] = 0
            self.current_id = bb_info['cat']


        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)

        # Match descriptors.
        matches = bf.knnMatch(descriptor_object, descriptor, 2)

        matches = np.array(matches)

        ## Apply Lowes ratio test
        good = []
        for m,n in matches:
          if m.distance < 0.4*n.distance:
             good.append(m)
        matches = good

        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)

        real_width = 0.2
        real_height = 0.2
        keypoints_3d = []
        keypoints_2d = []
        image_points = []
        object_points = []

        for i in range(len(matches)):
            object_index = matches[i].trainIdx
            image_index = matches[i].queryIdx
            keypoints_3d.append(keypoint_object[object_index])
            keypoints_2d.append(keypoints[image_index])

        for i in keypoints_2d:
            image_points.append((i.pt[0], i.pt[1],))

        for i in keypoints_3d:

            object_points.append((i.pt[0]*(real_width/image_width), i.pt[1]*(real_height/image_height), 0))

        if len(matches) <4:
            return None, None

        else:
            return np.array(object_points), np.array(image_points)

        # img_pts  = np.float32([kpA[m.queryIdx].pt for m in dmatches]).reshape(-1,2)
        # src_pts  = np.float32([kpB[m.trainIdx].pt for m in dmatches]).reshape(-1,2)

    # Estimates the pose with solvePnP, returns the sign pose in camera frame
    def estimate_pose(self, object_points, image_points, image_header):

        if self.tvec[0] == 0:
            retval, self.rvec, self.tvec = cv2.solvePnP(object_points, image_points, self.mtx, self.dist, flags = cv2.SOLVEPNP_ITERATIVE)
        else:
            retval, self.rvec, self.tvec = cv2.solvePnP(object_points, image_points, self.mtx, self.dist,
            rvec = self.rvec, tvec = self.tvec, flags = cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess= True)

        sign_pose = PoseStamped()

        sign_pose.header = image_header

        sign_pose.pose.position.x = tvec[0]
        sign_pose.pose.position.y = tvec[1]
        sign_pose.pose.position.z = tvec[2]

        sign_pose.pose.orientation = quaternion_from_euler(rvec[0],rvec[1],rvec[2])

        sign_pose

    # Publishes the pose stamped sign pose
    def publish_pose(self, sign_pose, bb_info):

        self.detected_sign_pub.publish(sign_pose)

    # Publishes the transform from map to detected sign
    def publish_transform(self, sign_pose, bb_info):
        trans = TransformStamped()

        category = bb_info['cat']

        category = category.replace(" ", "_")

        trans.header = sign_pose.Image_header
        trans.header.frame_id = 'map'
        trans.child_frame_id = 'detector/detected_sign_' + category

            # marker pose is in frame camera_link
        if not tf_buf.can_transform('map', sign_pose.header.frame_id, sign_pose.header.stamp, rospy.Duration(1)):
            rospy.logwarn('pose_estimation: No transform from %s to map', sign_pose.header.frame_id)
            return

        sign_transform = tf_buf.transform(sign_pose, 'map')

        trans.transform.translation.x = sign_transform.pose.position.x
        trans.transform.translation.y = sign_transform.pose.position.y
        trans.transform.translation.z = sign_transform.pose.position.z
        trans.tranform.rotation = sign_transform.pose.orientation

        self.broadcaster.sendTransform(trans)

    def callback(self, image):

        image_header = image.header

        # convert ros image to cv2
        try:
            cv_image = bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            print(e)

        # convert to rgb
        RGB_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # convert to pillow image
        PIL_image  = Img.fromarray(RGB_image)

        # REMOVE DISTORTION! WIP
        # h,w = PIL_image.shape[:2]
        # new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_parameters, (w,h), 1, (w,h))
        # dst = cv2.undistort(PIL_image, camera_matrix, distortion_parameters, None, new_camera_matrix)
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]

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
            bbs = detector.decode_output(out, self.confidence)

        bb_info = self.bounding_box(bbs, cv_image)

        # self.pose_estimation(cv_image, bb_info)

        kp_camera, des_camera = self.extract_features(cv_img, bb_info)

        object_points, image_points = self.match_features(kp_camera, des_camera)

        if object_points:

            sign_pose = self.estimate_pose(object_points, image_points, image_header)

            self.publish_pose(sign_pose, bb_info)

            self.publish_transform(sign_pose, bb_info)

        else:
            continue

def main(args):
  rospy.init_node('detection_node', anonymous=True)
  rospy.loginfo('detection node initiated')

  # Start Detector
  detector = sign_detection()

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

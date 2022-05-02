#!/usr/bin/env python

# An initial attempt for the detection node, I think it's better to convert it to a class (like in flight camp).
from ast import Global
import string
from sys import flags
from types import NoneType
from PIL import Image
import cv2
from cv2 import SOLVEPNP_ITERATIVE
from matplotlib import image
from matplotlib.pyplot import flag
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
from project.msg import Detection, DetectionArray
from detector import Detector
import os
import numpy as np
import time
import math
import random
# from msg import Detected

dir = os.path.abspath(os.getcwd())

categories = {0: "no_bicycle", 1: "airport", 2: "dangerous_curve_left",
                3: "dangerous_curve_right", 4: "follow_left",
                5: "follow_right", 6: "junction", 7: "no_heavy_truck",
                8: "no_parking", 9: "no_stopping_and_parking",
                10: "residential", 11: "narrows_from_left",
                12: "narrows_from_right", 13: "roundabout", 14: "stop"}

global tvec, rvec, currentid, refs
tvec = [0, 0, 0]
rvec = [0, 0, 0]
currentid = -1


def callback(Image):
    """Callback on the image from camera, processes one image at a time to avoid
        quing up 'old' images. It takes the raw image and runs the detector
        for bounding boxes and classification."""
    global bridge, Image_header

    Image_header = Image.header
    # convert ros image to cv2
    try:
        cv_image = bridge.imgmsg_to_cv2(Image, "bgr8")
    except CvBridgeError as e:
        print(e)

    # convert to rgb
    RGB_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

    # convert to pillow image

    # REMOVE DISTORTION! WIP
    # h,w ,_= RGB_image.shape
    # new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    # dst = cv2.undistort(RGB_image, mtx, dist, None, new_camera_matrix)
    # x, y, w, h = roi
    # dst = dst[y:y+h, x:x+w]

    dst = Img.fromarray(RGB_image)

    # What follows here is just some weird structuring of the data I had to do for it to fit into the demands of the model, not sure why.
    # But the list detect_images is what is passed to the model.
    detect_images = []
    torch_image, _ = detector.input_transform(dst, [])
    detect_images.append(torch_image)

    if detect_images:
        detect_images = torch.stack(detect_images)
        detect_images = detect_images.to(device)

    # We run the detector/model/network on the image here bbs is the decoded data
    #   bbs = detector(detect_images)
    with torch.no_grad():
        out = detector(detect_images)
        bbs = detector.decode_output(out, 0.98) #test 0.99??


        # Uncomment this part to test if it publishes the tranform for detected sign
        #publish_detection(bbs, timestamp)

        bounding_box(bbs, cv_image)


def reference_features():
    """ This function, takes all the cannonical pdf traffic signs and calculates their features and
        descriptors, this is done on init to avoid recalculation """

    # Initiate SIFT detector
    sift = cv2.xfeatures2d.SIFT_create()

    # structure {name1: {kp1:(), des1:()}, name2: {kp2:(), des2:()}}
    ref = {}
    file_name = {0: "no_bicycle", 1: "airport" , 2: "dangerous_curve_left", 3: "dangerous_curve_right", 4: "follow_left",
                5: "follow_right", 6: "junction", 7: "no_heavy_truck", 8: "no_parking", 9:"no_stopping_and_parking",
                10: "residential", 11: "narrows_from_left", 12: "narrows_from_right", 13: "roundabout", 14: "stop"}

    # import images
    for i in range(len(file_name)):
        sign_file_name = file_name[i]
        sign_name = categories[i]

        base_img = cv2.imread("/home/clemente/dd2419_ws/src/project/scripts/traffic_signs/" + sign_file_name  + ".jpg", cv2.IMREAD_COLOR)
        # convert cannonical image to gray scale
        base_gray = cv2.cvtColor(base_img, cv2.COLOR_BGR2GRAY)

        #base_gray = cv2.resize(base_gray, (0,0), fx=0.1, fy=0.1)
        (h,w) = base_gray.shape[:2]
        base_gray = cv2.resize(base_gray, (int(w/6.458), int(h/6.458)))
        base_gray = base_gray[150:base_gray.shape[0]-150, :]  #212

        #TA BORT SENARE
        (h,w) = base_gray.shape[:2]
        base_gray = cv2.resize(base_gray, (int(w), int(h)))

        b_height, b_width = base_gray.shape
        kp, des = sift.detectAndCompute(base_gray, None)

        ref[sign_name] = {'kp': kp, 'des': des, 'width': b_width, 'height': b_height}

    return ref


def bounding_box(detections, cv_image):
    """ Function to print bounding boxes and bounding box labels and publish the result in its own topic
        We also pass the image and bounding box info to the pose estimator """
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

    # Pick box with highest confidance
    if len(detections[0]) != 0:
        bb = max(detections[0], key=lambda j:j["score"])
        x = int(bb['x']) - int(bb['width']*0.05)
        y = int(bb['y']) - int(bb['height']*0.05)
        width = int(bb['width']) + int(bb['width']*0.05)
        height = int(bb['height']) + int(bb['height']*0.05)
        classification = categories[bb['category']]

        bb_info = {'x': x, 'y': y, 'width': width, 'height': height, 'category': classification, 'id': bb['category'], 'confidence': bb["score"]}
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
    """ Utility function to extract keypoints based on the matched points indexes """
    keypoints_3d = []
    keypoints_2d = []

    for i in range(len(matches)):
        object_index = matches[i].queryIdx
        image_index = matches[i].trainIdx
        keypoints_3d.append(kp_object[object_index])
        keypoints_2d.append(kp_image[image_index])

    return keypoints_3d, keypoints_2d


def object_keypoint_to_3d(object_keypoints, image_width, image_height):
    """ Utility function to transform the keypoints into 3D points for pose estimation """
    object_points = []
    real_width = 0.2
    real_height = 0.2
    for i in object_keypoints:
        #111object_points.append(((i.pt[0]-image_width/2)*(real_width/image_width), (i.pt[1]-image_height/2)*(real_height/image_height), 0))
        object_points.append((-(i.pt[1] - image_height/2)*(real_width/image_width), 0, (i.pt[0]-image_width/2)*(real_height/image_height)))

        #object_points.append((0, -i.pt[0]*(real_width/image_width), -i.pt[1]*(real_height/image_height)))

    return np.array(object_points)


def image_keypoint_to_2d(image_keypoints):
    """ Utility function to transform the keypoints into 2D points for pose estimation """
    image_points = []
    for i in image_keypoints:
        image_points.append((i.pt[0], i.pt[1],))
    return np.array(image_points)


def pose_estimation(camera_image, bb_info):
    """ Function that performs the pose estimation, it takes the camera image and bb info,
        it runns feature detection on the camera image and then matches the descriptors and keypoints
        in the camera image to those in the cannonical image. We then extract the best matches
        and use solvePnP to estimate the pose. """
    global Image_header, tvec, rvec, currentid, refs

    # convert camera image to gray scale
    cv_gray = cv2.cvtColor(camera_image, cv2.COLOR_BGR2GRAY)

    # Bounding box size
    x = bb_info['x']
    y = bb_info['y']
    width = bb_info['width']
    height = bb_info['height']

    #???????????????????????????
    if currentid != bb_info['id']:
        tvec[0] = 0
        currentid = bb_info['id']

    # Cropp image to only include bounding box info
    cropped_img = cv_gray[y:y+height, x:x+width]
    #cropped_img = cv_gray

    # Import the cannonical traffic sign based on detected class
    #base_img = cv2.imread(dir + "traffic_signs/" + bb_info['category']  + ".jpg", cv2.IMREAD_COLOR)
    #base_img = cv2.imread("/home/clemente/dd2419_ws/src/project/scripts/traffic_signs/" + bb_info['category']  + ".jpg", cv2.IMREAD_COLOR)

    # convert cannonical image to gray scale
    #base_gray = cv2.cvtColor(base_img, cv2.COLOR_BGR2GRAY)


    #(h,w) = base_gray.shape[:2]
    #base_gray = cv2.resize(base_gray, (int(w/6.458), int(h/6.458)))
    #base_gray = base_gray[150:base_gray.shape[0]-150, :]

    #TA BORT SENARE
    #(h,w) = base_gray.shape[:2]
    #base_gray = cv2.resize(base_gray, (int(w/10), int(h/10)))

    b_height, b_width = refs[bb_info['category']]['height'], refs[bb_info['category']]['width']
    # Initiate SIFT detector
    sift = cv2.xfeatures2d.SIFT_create()
    #orb = cv2.ORB_create()
    t = time.time()

    # find the keypoints and descriptors with SIFT for both images
    ####kp1, des1 = sift.detectAndCompute(base_gray, None)
    kp1 = refs[bb_info['category']]['kp']
    des1 = refs[bb_info['category']]['des']

    #kp1 = orb.detect(base_gray, None)
    #kp2 = orb.detect(cropped_img, None)
    #kp1 ,des1 = orb.compute(base_gray, kp1)
    #kp2, des2 = orb.compute(cropped_img, kp2)

    #print(time.time()-t)
    if cropped_img.size == 0 or type(cropped_img) == NoneType:
        return
    kp2, des2 = sift.detectAndCompute(cropped_img, None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    if len(kp2) == 0:
        return

    # Match descriptors with brute force
    matches = bf.knnMatch(des1,des2, 2)
    #matches = bf.match(des1,des2)
    if len(matches) == 0:
        return

    ##TEST FLANNNNNNN
    #FLANN_INDEX_KDTREE = 0
    #index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    #search_params = dict(checks = 50)

    #flann = cv2.FlannBasedMatcher(index_params, search_params)

    #matches = flann.knnMatch(des1,des2,k=2)


    matches = np.array(matches)
    ## Apply Lowes ratio test
    if matches.shape[1] == 1:
        return

    good = []

    # Find closest matches
    for m,n in matches:
        if m.distance < 0.6*n.distance:
            good.append(m)
    matches = good

    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    # Find the best matching keypoints from the matches
    keypoints_3d, keypoints_2d = keypoints_for_estimation(matches, kp1, kp2)

    # Convert keypoints into 3d object points
    object_points = object_keypoint_to_3d(keypoints_3d, b_width, b_height)

    # Convert keypoints into 2d image points
    image_points = image_keypoint_to_2d(keypoints_2d)
    for n in image_points:
        n[0] += x
        n[1] += y
    matches = np.array(matches)
    if matches.size == 0:
        return

    #print(image_points[0])
    #print(keypoints_2d[0].pt)
    # Find rotation and translation vectors for pose estimation
    #img3 = cv2.drawKeypoints(base_gray,keypoints_3d,cv2.DRAW_MATCHES_FLAGS_DEFAULT,color=(120,157,187))

    #img3 = cv2.drawMatches(base_gray,kp1,cropped_img,kp2,matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    #cv2.imwrite('aaaaaaaaaaaaaaa.jpg', img3)
    print("Matches: ", len(matches), "Min Distance; ", matches[0].distance)
    if len(matches) < 4:
        return

    #FOR OTHER FLAGS, REMOVE OTHERWISE
    #N,M = object_points.shape
    #object_points = np.ascontiguousarray(object_points[:,:2]).reshape((N,1,2))
    N,M = image_points.shape
    image_points = np.ascontiguousarray(image_points[:,:2]).reshape((N,1,2))
    object_points.astype('float32')
    #a= np.array([[0],[0],[0]])
    #b=np.array([[0],[0],[0.4]])
    #print(a)
    #a = np.ascontiguousarray(a, dtype=np.uint8)
    #b = np.ascontiguousarray(b, dtype=np.uint8)



    ####if tvec[0] == 0:
    ####    retval, rvec, tvec = cv2.solvePnP(object_points[:5], image_points[:5], mtx, dist,flags = cv2.SOLVEPNP_ITERATIVE)
####
    #####else:
    #### #   retval, rvec, tvec = cv2.solvePnP(object_points[:], image_points[:], mtx, dist,rvec= rvec,tvec=tvec, flags = cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess= True)
    ####retval, rvec, tvec = cv2.solvePnP(object_points[:5], image_points[:5], mtx, dist,rvec= rvec,tvec=tvec, flags = cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess= True)
    
     
    max_inliers = 0
    #rvec,tvec,inliers = None, None, None
    pairs = int(min(len(object_points), len(image_points)))
    if pairs > 4:
        for i in range(25):
            obj_samp, img_samp = zip(*random.sample(list(zip(object_points, image_points)), pairs))
            obj_samp = np.array([k for k in obj_samp])
            img_samp = np.array([j for j in img_samp])
            #if tvec[0] != 0:
            #    _, r_vec, t_vec, in_liers= cv2.solvePnPRansac(obj_samp, img_samp, mtx, dist, rvec=rvec,tvec=tvec,flags=SOLVEPNP_ITERATIVE, useExtrinsicGuess=True)
            #else:
            #    _, r_vec, t_vec, in_liers= cv2.solvePnPRansac(obj_samp, img_samp, mtx, dist,flags=SOLVEPNP_ITERATIVE)
            _, r_vec, t_vec, in_liers= cv2.solvePnPRansac(obj_samp, img_samp, mtx, dist,flags=SOLVEPNP_ITERATIVE)

            if in_liers is not None:
                n_in = len(in_liers)
                if n_in > max_inliers:
                    max_inliers = n_in
                    print(max_inliers)
                    rvec, tvec, inliers = r_vec, t_vec, in_liers
            #else:
            #    rvec, tvec, inliers = r_vec, t_vec, in_liers
            #    break


    #rvec[2] = 0
    #rvec[0] = -math.pi/2
    if tvec[0] > 2 or tvec[1] > 2 or tvec[2] > 3:# or type(inliers) == NoneType:
        tvec[0] = 0
        return
    if tvec[0]**2 + tvec[1]**2 +tvec[2]**2< 0.1**2:
        tvec[0] = 0
        return
    if tvec[2] < 0: #outlier :))))))
        tvec[0] = 0
        return
    if math.isnan(rvec[0]) or math.isnan(rvec[0]) or math.isnan(rvec[0]):
        tvec[0] = 0
        return
    #rvec[2] = 0

    #inliers = np.asarray(inliers).reshape(-1)
    #print(inliers)
    #EXTRA :)
    #ke = cv2.KeyPoint(x = 0, y = 1, _size = 1)
    #print(ke.pt)
    #keypoints_2d[0].pt = ke.pt
    #print(keypoints_2d[0].pt)
    #img_kp = cv2.drawKeypoints(base_gray,keypoints_3d,cv2.DRAW_MATCHES_FLAGS_DEFAULT,color=(120,157,187))


    #cv_gray = cv2.circle(cv_gray, (int(image_points[0][0]),int(image_points[0][1])), radius=0, color=(0, 0, 255), thickness=-1)
    #cv_gray = cv2.circle(cv_gray, (int(image_points[1][0]),int(image_points[1][1])), radius=0, color=(0, 0, 255), thickness=-1)
    #print(cv_gray.shape)
    #cv2.imwrite('aaa.png', cv_gray)
    #time.sleep(7)
    #print("Printed!")
    #print(keypoints_3d[0].pt)
    #print("--------")
    #print(keypoints_3d[1].pt)
    #print("prriiiinted")

    sign_pose = PoseStamped()
    sign_pose.header = Image_header
    sign_pose.header.frame_id = Image_header.frame_id
    sign_pose.header.stamp = Image_header.stamp
    #print(tvec)
    #sign_pose.pose.position.x = tvec[2]
    #sign_pose.pose.position.y = -tvec[0]
    #sign_pose.pose.position.z = -tvec[1]
    sign_pose.pose.position.x = tvec[0]
    sign_pose.pose.position.y = tvec[1]
    sign_pose.pose.position.z = tvec[2]
    x, y, z, w = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
    sign_pose.pose.orientation.x = x
    sign_pose.pose.orientation.y = y
    sign_pose.pose.orientation.z = z
    sign_pose.pose.orientation.w = w

    publish_pose(sign_pose, bb_info['id'], bb_info["category"], bb_info["confidence"])


def publish_pose(sign_pose, sign_id, category, confidence):
    """ Creates the Transform from camera link to the detected sign and publishes to tf. Also
        creates a Detected_msg and DetectedArray_msg to post the pose for localization. """

    Detected_msg = Detection()
    DetectedArray_msg = DetectionArray()
    trans = TransformStamped()

    Detected_msg.header.frame_id = sign_pose.header.frame_id
    # trans.header.frame_id = 'map'
    Detected_msg.header.stamp = sign_pose.header.stamp
    trans.header.stamp = sign_pose.header.stamp

    trans.header.frame_id = 'cf1/camera_link'
    # trans.header.frame_id = 'map'
    trans.child_frame_id = 'detector/detectedsign_' + category
    sign_pose.header.frame_id = "cf1/camera_link"

    #   # marker pose is in frame camera_link
    #if not tf_buf.can_transform('map', 'cf1/camera_link', sign_pose.header.stamp, rospy.Duration(1)):
    #   rospy.logwarn('pose_estimation: No transform from %s to map', sign_pose.header.frame_id)
    #   print("heeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeej ingen transform :(")
    #   return

    #sign_transform = tf_buf.transform(sign_pose, 'map')
    Detected_msg.pose.header.frame_id = sign_pose.header.frame_id
    Detected_msg.pose.pose.position.x = sign_pose.pose.position.x
    Detected_msg.pose.pose.position.y = sign_pose.pose.position.y
    Detected_msg.pose.pose.position.z = sign_pose.pose.position.z
    trans.transform.translation.x = sign_pose.pose.position.x
    trans.transform.translation.y = sign_pose.pose.position.y
    trans.transform.translation.z = sign_pose.pose.position.z

    Detected_msg.pose.pose.orientation.x = sign_pose.pose.orientation.x
    Detected_msg.pose.pose.orientation.y = sign_pose.pose.orientation.y
    Detected_msg.pose.pose.orientation.z = sign_pose.pose.orientation.z
    Detected_msg.pose.pose.orientation.w = sign_pose.pose.orientation.w
    trans.transform.rotation.x = sign_pose.pose.orientation.x
    trans.transform.rotation.y = sign_pose.pose.orientation.y
    trans.transform.rotation.z = sign_pose.pose.orientation.z
    trans.transform.rotation.w = sign_pose.pose.orientation.w

    broadcaster.sendTransform(trans)
    #Publish message
    Detected_msg.id  = sign_id
    Detected_msg.confidence = confidence
    DetectedArray_msg.detections = [Detected_msg]

    """ Publish to /detected_sign """
    detected_pub.publish(DetectedArray_msg)

    """ Publish to /intruder_detection_sign to perform intruder detection """
    intruder_det_pub.publish(DetectedArray_msg)

# Init node
rospy.init_node('detect_node')

# Init publisher
image_pub = rospy.Publisher("/bbox", Image, queue_size=1)
detected_pub = rospy.Publisher("/detected_sign", DetectionArray, queue_size=10)
intruder_det_pub = rospy.Publisher("/intruder_detection_sign", DetectionArray, queue_size=10)

# Init TF
tf_buf   = tf2_ros.Buffer()
tf_listner  = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()

# Init detector with trained weights and set to evaluation mode
is_rocm_pytorch = True
device = torch.device('cuda')
detector = Detector().to(device)
#dataloader = utils.load_model(detector, "/home/miguelclemente/dd2419_ws/src/part3/scripts/det_2022-02-20_19-48-10-524174.pt" , device)
#path = dir + "/scripts/det_2022-02-20_19-48-10-524174.pt"
path = "/home/clemente/dd2419_ws/src/project/scripts/det_2022-04-01_10-48-59-587336.pt"
dataloader = utils.load_model(detector, path, device)
refs = reference_features()
detector.eval()
bridge = CvBridge()
#min
#mtx = np.array([[219.72672444 ,  0.     ,    327.75533564],
#                [  0.     ,    217.82483989 ,240.2481643 ],
#                [  0.   ,        0.    ,       1.        ]])

# Distortion matrix
#dist = np.array([ 0.14543323, -0.12488487,  0.02666364, -0.00334833,  0.0199592 ])

#min
#dist = np.array([ 0.20962176 ,-0.22188961 ,-0.00044311 , 0.00040148  ,0.05630106])

#dist = np.array([ 0,0,0,0,0])

#Teos nya
#mtx =  np.array([[207.89466123 , 0.  ,   327.20867249],
#                   [ 0.  ,   205.19301437, 242.26158286],
#                   [ 0. ,     0.    ,  1.    ]])
#dist = np.array(  [0.17486646 ,-0.1696708, -0.00025329, 0.00033005,0.03769554]   )

#min fast i mm
dist = np.array([ 2.43077029e-01 ,-2.88500502e-01 ,-3.30439375e-04  ,6.91331506e-05 ,8.45047348e-02])
mtx = np.array([[231.45889158 ,  0.   ,      326.96473621],
 [  0.       ,  229.12660112 ,236.31074878],
 [  0.     ,      0.       ,    1.        ]])

print("Running...")


def caminfo(caminfo_msg):
    global camera_matrix, distortion_parameters

    camera_matrix = caminfo_msg.K
    distortion_parameters = caminfo_msg.D


def main():
    image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, callback,
                                 queue_size=1, buff_size=2**24)
    rospy.spin()


if __name__ == '__main__':
    caminfo_sub = rospy.Subscriber("/cf1/camera/camera_info", CameraInfo,
                                   caminfo)

    main()

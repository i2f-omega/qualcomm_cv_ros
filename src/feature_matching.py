#!/usr/bin/env python2
"""
Detects features in a ROS image stream
using ORB or SIFT feature detection

Matches the features to a reference image

Optionally: Performs histogram equalization on
greyscale images prior to detecting features
"""

# Standard dependencies
import sys
import os
import math
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Will only do histogram equalization on greyscale images
use_hist_equal = True
min_good_matches = 5
ORB_match_cutoff = 35.0
SIFT_match_cutoff = 200.0
match_alg = "SIFT" # "SIFT" or "ORB"
class FeatureMatching():

    def __init__(self):

        sub_img_topic = "/hires/image_raw"
        pub_matches_topic = "/hires/feature_matches"
        pub_trans_topic = "/hires/feature_transform"
        image_path = "/home/kcoble/catkin_ws/src/qualcomm_cv_ros/include/pictures/loft_lamp_grey.png"

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publishers
        self.pub_matches = rospy.Publisher(pub_matches_topic, Image, queue_size=10)
        self.pub_trans = rospy.Publisher(pub_trans_topic, Image, queue_size=10)

        if match_alg == "ORB": # Initiate ORB detector & BFMatcher object
            self.matcher = cv2.ORB_create(nfeatures=1500)
            self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        elif match_alg == "SIFT": # Initiate SIFT detector & BFMatcher object
            self.matcher = cv2.xfeatures2d.SIFT_create()
            self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

        else:
            print("INVALID FEATURE MATCHING ALGORITHM")

        # create BFMatcher object

        # Read the reference image, find the keypoints and compute the descriptors with ORB
        ref_color = cv2.imread(image_path)
        # self.ref_img = cv2.resize(ref_color, (1920,1080))
        self.ref_img = cv2.resize(ref_color, (0,0), fx = 1.0, fy = 1.0)
        self.ref_grey = cv2.cvtColor(self.ref_img, cv2.COLOR_BGR2GRAY)
        self.ref_kp, self.ref_des = self.matcher.detectAndCompute(self.ref_img, None)

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)


    def img_callback(self, msg):
        try:
            if msg.encoding == "mono8": # Greyscale highres stream from Qualcomm
                stream = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if use_hist_equal: # Can only equalize a greyscale image
                    # stream = cv2.equalizeHist(stream)
                    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(24,24))
                    stream = clahe.apply(stream)
            else: # Color highres stream from Qualcomm
                stream = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # find the keypoints and compute the descriptors
            kp, des = self.matcher.detectAndCompute(stream, None)

            # Match descriptors.
            matches = self.bf.match(self.ref_des, des)
            # # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)

            good = []
            if match_alg == "ORB": # Initiate ORB detector & BFMatcher object
                for m in matches:
                    if m.distance < ORB_match_cutoff:
                        good.append(m)
                    # else: # Only use if sorted
                    #     break
            elif match_alg == "SIFT": # Initiate SIFT detector & BFMatcher object
                for m in matches:
                    if m.distance < SIFT_match_cutoff:
                        good.append(m)
                    # else: # Only use if sorted
                    #     break
            print("good matches:", len(good))
            # good = matches[:30]

            if len(good) >= min_good_matches:

                src_pts = np.float32([self.ref_kp[m.queryIdx].pt for m in good]).reshape(-1,1,2)
                dst_pts = np.float32([kp[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                print("M", M)

                # matchesMask = mask.ravel().tolist()
                h, w, d = self.ref_img.shape
                print("h", h)
                print("w", w)
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)
                print(dst)

                trans_img = cv2.polylines(stream,[np.int32(dst)],True,255,3, cv2.LINE_AA)
                self.pub_trans.publish(self.bridge.cv2_to_imgmsg(trans_img, "passthrough"))
            else:
                print("Not enough good matches")
                print(len(good), " / ", min_good_matches)

            # Publish matches no matter what
            if msg.encoding == "mono8": # Greyscale highres stream from Qualcomm
                # Draw first __ matches.
                matched_img = cv2.drawMatches(self.ref_grey,self.ref_kp,stream,kp,good, outImg = None, flags=2)
            else:
                # Draw first __ matches.
                matched_img = cv2.drawMatches(self.ref_img,self.ref_kp,stream,kp,good, outImg = None, flags=2)

            self.pub_matches.publish(self.bridge.cv2_to_imgmsg(matched_img, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('feature_matching', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    FM = FeatureMatching()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
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

class FeatureMatching():

    def __init__(self):
        # Load params from launch file
        param = rospy.search_param("subscribed_image_topic")
        sub_img_topic = rospy.get_param(param)
        param = rospy.search_param("matches_image_topic")
        pub_matches_topic = rospy.get_param(param)
        param = rospy.search_param("boxed_image_topic")
        pub_bound_box_topic = rospy.get_param(param)

        param = rospy.search_param("detection_algorithm")
        self.detect_alg = rospy.get_param(param)
        param = rospy.search_param("use_hist_equal")
        self.use_hist_equal = rospy.get_param(param)

        param = rospy.search_param("minimum_good_matches")
        self.min_good_matches = rospy.get_param(param)
        param = rospy.search_param("ORB_match_cutoff")
        self.ORB_match_cutoff = rospy.get_param(param)
        param = rospy.search_param("SIFT_match_cutoff")
        self.SIFT_match_cutoff = rospy.get_param(param)

        param = rospy.search_param("image_path")
        image_path = rospy.get_param(param)
        param = rospy.search_param("image_rescale")
        image_rescale = rospy.get_param(param)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publishers
        self.pub_matches = rospy.Publisher(pub_matches_topic, Image, queue_size=10)
        self.pub_bound_box = rospy.Publisher(pub_bound_box_topic, Image, queue_size=10)

        if self.detect_alg == "ORB": # Initiate ORB detector & BFMatcher object
            self.detector = cv2.ORB_create(nfeatures=1500)
            self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        elif self.detect_alg == "SIFT": # Initiate SIFT detector & BFMatcher object
            self.detector = cv2.xfeatures2d.SIFT_create()
            self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
        else:
            print("INVALID FEATURE MATCHING ALGORITHM")

        if self.use_hist_equal: # Initialize adaptive histogram equalization
            self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(24,24))

        # Read the reference image, find the keypoints and compute the descriptors with ORB
        self.ref_img = cv2.imread(image_path)
        self.ref_img = cv2.resize(self.ref_img, (0,0), fx = image_rescale, fy = image_rescale)
        self.ref_grey = cv2.cvtColor(self.ref_img, cv2.COLOR_BGR2GRAY)
        self.ref_kp, self.ref_des = self.detector.detectAndCompute(self.ref_img, None)

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)


    def img_callback(self, msg):
        try:
            if msg.encoding == "mono8": # Greyscale highres stream from Qualcomm
                stream = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if self.use_hist_equal: # Can only equalize a greyscale image
                    # stream = cv2.equalizeHist(stream)
                    stream = self.clahe.apply(stream)
            else: # Color highres stream from Qualcomm
                stream = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # find the keypoints and compute the descriptors
            kp, des = self.detector.detectAndCompute(stream, None)

            # Match descriptors.
            matches = self.bf.match(self.ref_des, des)
            # # Sort them in the order of their distance.
            # matches = sorted(matches, key = lambda x:x.distance)

            good = []
            if self.detect_alg == "ORB": # Initiate ORB detector & BFMatcher object
                for m in matches:
                    if m.distance < self.ORB_match_cutoff:
                        good.append(m)
                    # else: # Only use if sorted
                    #     break
            elif self.detect_alg == "SIFT": # Initiate SIFT detector & BFMatcher object
                for m in matches:
                    if m.distance < self.SIFT_match_cutoff:
                        good.append(m)
                    # else: # Only use if sorted
                    #     break
            print("good matches:", len(good))
            # good = matches[:30]

            if len(good) >= self.min_good_matches:

                src_pts = np.float32([self.ref_kp[m.queryIdx].pt for m in good]).reshape(-1,1,2)
                dst_pts = np.float32([kp[m.trainIdx].pt for m in good]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                # matchesMask = mask.ravel().tolist()
                h, w, d = self.ref_img.shape
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)

                bound_box_img = cv2.polylines(stream,[np.int32(dst)],True,255,3, cv2.LINE_AA)
                self.pub_bound_box.publish(self.bridge.cv2_to_imgmsg(bound_box_img, "passthrough"))
            else:
                print("Not enough good matches")
                print(len(good), " / ", self.min_good_matches)

            # Draw and publish "good" matches
            if msg.encoding == "mono8": # Greyscale highres stream from Qualcomm
                matched_img = cv2.drawMatches(self.ref_grey,self.ref_kp,stream,kp,good, outImg = None, flags=2)
            else:
                matched_img = cv2.drawMatches(self.ref_img,self.ref_kp,stream,kp,good, outImg = None, flags=2)

            self.pub_matches.publish(self.bridge.cv2_to_imgmsg(matched_img, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('feature_matching', anonymous=True)
    rospy.loginfo("Successful initialization of node")

    FM = FeatureMatching()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
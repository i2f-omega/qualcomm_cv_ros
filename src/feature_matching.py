#!/usr/bin/env python2
"""
Detects features in a ROS image stream
using ORB feature detection

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

# Choose one or neither
# If both, it will use standard
use_hist_equal = True
num_matches = 15

class HistogramEqualization():

    def __init__(self):

        sub_img_topic = "/hires/image_raw"
        pub_img_topic = "/hires/matched_features"

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publisher
        self.pub = rospy.Publisher(pub_img_topic, Image, queue_size=10)

        # Initiate STAR detector
        self.orb = cv2.ORB_create()

        # create BFMatcher object
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Read the reference image, find the keypoints and compute the descriptors with ORB
        ref_color = cv2.imread('/home/kcoble/catkin_ws/src/qualcomm_cv_ros/include/loft_color.jpg')
        self.ref_img = cv2.resize(ref_color, (1920, 1080))
        self.ref_grey = cv2.cvtColor(self.ref_img, cv2.COLOR_BGR2GRAY)
        self.ref_kp, self.ref_des = self.orb.detectAndCompute(self.ref_img, None)

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)


    def img_callback(self, msg):
        try:

            if msg.encoding == "mono8": # Greyscale highres stream from Qualcomm
                stream = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if use_hist_equal: # Can only equalize a greyscale image
                    stream = cv2.equalizeHist(stream)
            else: # Color highres stream from Qualcomm
                stream = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # find the keypoints and compute the descriptors with ORB
            kp, des = self.orb.detectAndCompute(stream, None)

            # Match descriptors.
            matches = self.bf.match(self.ref_des, des)
            # Sort them in the order of their distance.
            matches = sorted(matches, key = lambda x:x.distance)

            if msg.encoding == "mono8": # Greyscale highres stream from Qualcomm
                # Draw first __ matches.
                matched_img = cv2.drawMatches(self.ref_grey,self.ref_kp,stream,kp,matches[:num_matches], outImg = None, flags=2)
            else:
                # Draw first __ matches.
                matched_img = cv2.drawMatches(self.ref_img,self.ref_kp,stream,kp,matches[:num_matches], outImg = None, flags=2)

            self.pub.publish(self.bridge.cv2_to_imgmsg(matched_img, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('feature_matching', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    HE = HistogramEqualization()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
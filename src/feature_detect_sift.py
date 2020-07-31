#!/usr/bin/env python2
"""
Detects features in a ROS image stream
using ORB feature detection

Optionally: Performs histogram equalization
prior to detecting features
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

class FeatureDetection():

    def __init__(self):

        sub_img_topic = "/hires/image_raw"
        pub_img_topic = "/hires/features_SIFT"

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publisher
        self.pub = rospy.Publisher(pub_img_topic, Image, queue_size=10)

        # Initiate SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)
        rospy.sleep(0.5) # Delay briefly to allow subscribers to find messages


    def img_callback(self, msg):
        print("callback")
        try:
            if msg.encoding == "mono8": # Greyscale highres stream from Qualcomm
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if use_hist_equal: # Can only equalize a greyscale image
                    image = cv2.equalizeHist(image)
            else: # Color highres stream from Qualcomm
                image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # find the keypoints and compute the descriptors with ORB
            kp, des = self.sift.detectAndCompute(image, None)

            # draw only keypoints location,not size and orientation
            image = cv2.drawKeypoints(image,kp,outImage = None,color=(0,255,0), flags=0)

            self.pub.publish(self.bridge.cv2_to_imgmsg(image, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('feature_detection', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    FD = FeatureDetection()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
#!/usr/bin/env python2
"""
Performs historgam equalization of a
greyscale ROS image stream

This script is mostly as a demo and we should
use the histogram equalization functions in
our other nodes as necessary
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
use_hist_equal = False
use_clahe_equalization = True

class HistogramEqualization():

    def __init__(self):

        sub_img_topic = "/hires/image_raw"
        pub_img_topic = "/hires/equalized"

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publisher
        self.pub = rospy.Publisher(pub_img_topic, Image, queue_size=10)

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)
        rospy.sleep(0.5) # Delay briefly to allow subscribers to find messages


    def img_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if use_hist_equal:
                image = cv2.equalizeHist(image)
            elif use_clahe_equalization:
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(24,24))
                image = clahe.apply(image)

            self.pub.publish(self.bridge.cv2_to_imgmsg(image, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('image_stream_rotate', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    HE = HistogramEqualization()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
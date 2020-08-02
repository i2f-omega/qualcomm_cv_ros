#!/usr/bin/env python2
"""
Converts the color image stream of a ROS
image stream to bgr8 and republishes it

Encoding of bgr8 is necessary for the
color image stream coming from the
ModalAI Qualcomm Flight Pro because
it comes in encoded in a format that
cannot be visualized in RVIZ

This script is just a demo and we should
use the encoding configuration
(desired_encoding="bgr8")
in our other nodes as necessary
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



class ColorStream():

    def __init__(self):

        sub_img_topic = "/hires/image_raw"
        pub_img_topic = "/hires/color_repub"

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publisher
        self.pub = rospy.Publisher(pub_img_topic, Image, queue_size=10)

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)
        rospy.sleep(0.5) # Delay briefly to allow subscribers to find messages


    def img_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.pub.publish(self.bridge.cv2_to_imgmsg(image, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('color_image_stream', anonymous=True)
    rospy.loginfo("Successful initialization of node")

    CS = ColorStream()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
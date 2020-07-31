#!/usr/bin/env python2


# Test dependencies
import sys
import os
import math
import rospy
import numpy as np

import tf2_ros
import tf_conversions
from tf.transformations import quaternion_from_euler, euler_from_quaternion, inverse_matrix

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

print("Using python version:")
print(sys.version)

print("Using OpenCV version:")
print(cv2.__version__)
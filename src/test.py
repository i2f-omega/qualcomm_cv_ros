#!/usr/bin/env python2


# Standard dependencies
import sys
import os
import math
import rospy
import numpy as np

# For OpenCV
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# For aruco markers
import yaml
from geometry_msgs.msg import Quaternion, Transform, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion, inverse_matrix
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix


print("Using python version:")
print(sys.version)

print("Using OpenCV version:")
print(cv2.__version__)
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

# For OpenCV
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# For aruco markers
from Marker import Marker
import yaml
from geometry_msgs.msg import Quaternion, Transform, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion, inverse_matrix
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix

class ArucoLocalization():

    def __init__(self):

        sub_img_topic = "/hires/image_raw"
        aruco_image_topic = "/detected_markers"
        pub_pose_topic = "/qualcomm/pose"

        cam_calibration_path = "/home/kcoble/catkin_ws/src/qualcomm_cv_ros/include/default_hires_calibration.yaml"
        marker_map_path = "/home/kcoble/catkin_ws/src/qualcomm_cv_ros/include/marker_map_loft.yaml"
        aruco_dictionary = cv2.aruco.DICT_4X4_250

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publisher
        self.pose_pub = rospy.Publisher(pub_pose_topic, PoseStamped, queue_size=10)
        self.im_pub = rospy.Publisher(aruco_image_topic, Image, queue_size=1)

        # Initialize aruco details
        self.cam_mtx, self.distort_mtx = self.loadCoefficients(cam_calibration_path)
        self.marker_set, self.marker_dict = self.load_markers(marker_map_path)
        self.dictionary = cv2.aruco.Dictionary_get(aruco_dictionary)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Initialize transform from camera to qualcomm
        tmat = translation_matrix((0.1, 0, 0.2))
        qmat = quaternion_matrix(quaternion_from_euler(-1.57, 0, -1.57))
        self.tf_cam2baselink = inverse_matrix(np.dot(tmat, qmat))

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)#, queue_size = 1)
        rospy.sleep(0.5) # Delay briefly to allow subscribers to find messages



    def img_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            image = cv2.resize(image, (640,480)) # Temporary until camera is calibrated properly

            ids, corners = self.detectMarkersInMap(image)
            pose, image = self.updatePoseFromMarkers(ids, corners, image)
            self.im_pub.publish(self.bridge.cv2_to_imgmsg(image, "passthrough"))

            if pose:
                qualcomm_pose = PoseStamped()
                qualcomm_pose.pose = pose
                qualcomm_pose.header.frame_id = "map"
                qualcomm_pose.header.stamp = rospy.Time.now()
                self.pose_pub.publish(qualcomm_pose)

        except CvBridgeError as e:
            print(e)


    def updatePoseFromMarkers(self, ids, corners, image):

        if ids:
            xs, ys, yaws = [], [], []
            for i, idx in enumerate(ids):

                rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_dict[str(idx[0])].len, self.cam_mtx, self.distort_mtx)
                rvec = rvec[0][0]
                tvec = tvec[0][0]

                tmat = translation_matrix((tvec[0], tvec[1], tvec[2]))
                qmat = np.zeros((4,4))
                qmat[:3, :3], _ = cv2.Rodrigues(rvec)
                qmat[3,3] = 1.

                tf_cam2mark = np.dot(tmat, qmat)
                tf_mark2cam = inverse_matrix(tf_cam2mark)

                marker = self.marker_dict[str(idx[0])]

                tf_map2cam = np.dot(marker.tf_mat, tf_cam2mark)
                # trans = translation_from_matrix(tf_map2cam)
                rot = euler_from_quaternion(quaternion_from_matrix(tf_map2cam))

                tf_map2baselink = np.dot(tf_map2cam, self.tf_cam2baselink)

                baselink_pose = Pose()
                baselink_pose.position = Vector3(*translation_from_matrix(tf_map2baselink))
                baselink_pose.orientation = Quaternion(*quaternion_from_matrix(tf_map2baselink))

                print(marker.id)
                print("cam pose", baselink_pose)

                # Draw axis on marker and publish image
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                image = cv2.aruco.drawAxis(image, self.cam_mtx, self.distort_mtx, rvec, tvec, 0.1)

            """
            NOTE
            Still need to take average of poses when multiple markers are detected
            """

            return baselink_pose, image
        else:
            return None, image


    def detectMarkersInMap(self, image):
        """
        Detects all markers in camera view
        and returns lists of ids and corners
        of those that are included in the marker_map.yaml
        """
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
            map_ids = []
            map_corners = []
            for i, idx in enumerate(ids):
                if idx[0] in self.marker_set:
                    map_ids.append(idx)
                    map_corners.append(corners[i])
            return map_ids, map_corners
        except:
            print("No aruco markers detected")
            return None, None

    @staticmethod
    def load_markers(path):
        """
        Loads a yaml file containing marker definitions
        Returns a list of marker ids and a dictionary
        of corresponding Marker() objects
        """
        marker_dict = {}
        marker_list = []
        with open(path) as f:
            raw_markers = yaml.load(f)
            for marker in raw_markers:
                id_  = raw_markers[marker][0]
                len_ = raw_markers[marker][1]

                x = raw_markers[marker][2][0]
                y = raw_markers[marker][2][1]
                z = raw_markers[marker][2][2]

                rot_x = raw_markers[marker][3][0]
                rot_y = raw_markers[marker][3][1]
                rot_z = raw_markers[marker][3][2]

                marker_list.append(id_)
                marker_dict[str(id_)] = Marker(id_, len_, x, y, z, rot_x, rot_y, rot_z)

        return set(marker_list), marker_dict

    @staticmethod
    def loadCoefficients(path):
        """
        Loads a yaml file containing camera calibration values
        generated by calibration/camera_calibration.py
        """
        with open(path) as f:
            calibration_data = yaml.load(f)
            #Camera matrix and distortion
            cam_mtx     = calibration_data.get("camera_matrix")
            distort_mtx = calibration_data.get("dist_coeff")
            cam_mtx     = np.asarray(cam_mtx)
            distort_mtx = np.asarray(distort_mtx)
        return cam_mtx, distort_mtx

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('aruco_localization', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    AL = ArucoLocalization()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()
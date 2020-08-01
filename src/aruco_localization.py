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
from geometry_msgs.msg import Quaternion, Transform, Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, inverse_matrix
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix

class ArucoLocalization():

    def __init__(self):

        sub_img_topic = "/hires/image_raw"
        pub_pose_topic = "/qualcomm/pose"

        cam_calibration_path = "/home/kcoble/catkin_ws/src/qualcomm_cv_ros/include/default_hires_calibration.yaml"
        marker_map_path = "/home/kcoble/catkin_ws/src/qualcomm_cv_ros/include/marker_map_loft.yaml"
        aruco_dictionary = cv2.aruco.DICT_4X4_250

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publisher
        self.pub = rospy.Publisher(pub_pose_topic, PoseStamped, queue_size=10)
        self.im_pub = rospy.Publisher("image_repub", Image, queue_size=1)

        # Initialize aruco details
        self.cam_mtx, self.distort_mtx = self.loadCoefficients(cam_calibration_path)
        self.marker_set, self.marker_dict = self.load_markers(marker_map_path)
        self.dictionary = cv2.aruco.Dictionary_get(aruco_dictionary)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback, queue_size = 1)
        rospy.sleep(0.5) # Delay briefly to allow subscribers to find messages



    def img_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            image = cv2.resize(image, (640,480)) # Temporary until camera is calibrated properly
            self.im_pub.publish(self.bridge.cv2_to_imgmsg(image, "passthrough"))

            ids, corners = self.detectMarkersInMap(image)
            self.updatePoseFromMarkers(ids, corners)




            temp_pub = PoseStamped()
            temp_pub.pose = self.marker_dict["2"].pose
            temp_pub.header.frame_id = "map"
            temp_pub.header.stamp = rospy.Time.now()

            self.pub.publish(temp_pub)

        except CvBridgeError as e:
            print(e)


    def updatePoseFromMarkers(self, ids, corners):
        """
        NOTE:
        For now I am ignoring 6 DOF pose and am
        only looking at rotation about the z-axis
        This assumes markers are all vertically
        upright and that we only need x, y, and yaw
        pose of the rover
        """
        if ids:
            xs, ys, yaws = [], [], []
            for i, idx in enumerate(ids):

                rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_dict[str(idx[0])].len, self.cam_mtx, self.distort_mtx)
                rvec = rvec[0][0]
                tvec = tvec[0][0]

                tmat = translation_matrix((tvec[0], tvec[1], tvec[2]))
                qmat = quaternion_matrix(quaternion_from_euler(rvec[0], rvec[1], rvec[2]))
                tf_cam2mark = np.dot(tmat, qmat)
                tf_mark2cam = inverse_matrix(tf_cam2mark)

                marker = self.marker_dict[str(idx[0])]

                tf_map2cam = np.dot(marker.tf_mat, tf_cam2mark)
                # trans = translation_from_matrix(tf_map2cam)
                trans = translation_from_matrix(tf_map2cam)
                rot = euler_from_quaternion(quaternion_from_matrix(tf_mark2cam))
                print(marker.id)
                print("tv", tvec)
                print("rv",rvec)
                print("t", trans)
                print("r", rot)

                # trans = Transform()
                # trans.translation.x = tvec[0]
                # trans.translation.y = tvec[1]
                # trans.translation.z = tvec[2]
                # trans.rotation = Quaternion(*quaternion_from_euler(
                #     rvec[0], rvec[1], rvec[2]))

                # print(trans)

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
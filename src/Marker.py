#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import translation_matrix, quaternion_matrix

class Marker():
    """
    Class for defining an aruco marker
    to be used in ArucoLocalization()
    """
    def __init__(self, id_, len_, x, y, z, rot_x, rot_y, rot_z):
        """
        :param id: marker id #
        :type id: int
        :param len_: marker length (incl. black boundary) in [m]
        :type id: float
        :param x, y, z, rot_x, rot_y, rot_z: 6DOF pose of the marker in [m] or [rad]
        :type x, y, z, rot_x, rot_y, rot_z: floats
        """
        self.id = id_
        self.len = len_
        self.pose = Pose()
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z
        rot_x += math.pi/2
        rot_y += 0
        rot_z += math.pi/2

        self.pose.orientation = Quaternion(*quaternion_from_euler(rot_x, rot_y, rot_z))
        tmat = translation_matrix((x, y, z))
        qmat = quaternion_matrix(quaternion_from_euler(rot_x, rot_y, rot_z))
        self.tf_mat = np.dot(tmat, qmat)

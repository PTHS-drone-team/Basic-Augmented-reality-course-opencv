import sys

import cv2 as cv
from cv2 import aruco
import numpy as np


class DistanceAruco:

    def __init__(self, calib_data_path, MARKER_SIZE, typeAruco, direction_change_value, speed_ch_3, cap):

        self.calib_data_path = calib_data_path

        # calib_data_path = "../calib_data_asus/MultiMatrix.npz"

        self.calib_data = np.load(self.calib_data_path)

        self.cam_mat = self.calib_data["camMatrix"]
        self.dist_coef = self.calib_data["distCoef"]
        self.r_vectors = self.calib_data["rVector"]
        self.t_vectors = self.calib_data["tVector"]

        self.MARKER_SIZE = MARKER_SIZE

        self.marker_dict = aruco.Dictionary_get(typeAruco)

        self.param_markers = aruco.DetectorParameters_create()

        self.direction_change_value = direction_change_value

        self.speed_ch_3 = speed_ch_3

    def get_frame(self, frame):
        self.frame = frame
        self.gray_frame = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)

        return self.gray_frame

    def get_marker_corners(self, frame):
        self.marker_corners, self.marker_IDs, reject = aruco.detectMarkers(
            self.get_frame(frame), self.marker_dict, parameters=self.param_markers
        )
        return self.marker_corners

    def get_distance(self, marker_corners):
        self.marker_corners = marker_corners
        if self.marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                self.marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef
            )
            total_markers = range(0, self.marker_IDs.size)
            for ids, corners, i in zip(self.marker_IDs, self.marker_corners, total_markers):
                # cv.polylines(
                #     self.frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                # )

                self.corners = corners.reshape(4, 2)
                self.corners = corners.astype(int)
                self.top_right = corners[0][0].ravel()
                self.top_left = corners[0][1].ravel()
                self.bottom_right = corners[0][2].ravel()
                self.bottom_left = corners[0][3].ravel()

                self.distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )

                return round(self.distance, 2)
        else:
            self.distance = None
            return self.distance

    def get_command_to_ch_3(self):
        self.d = 10
        if self.distance is not None:
            if self.distance < self.direction_change_value:
                return 1500 - self.speed_ch_3
            elif self.distance >= self.direction_change_value and self.distance <= self.direction_change_value + self.d:
                return 1500
            elif self.distance > self.direction_change_value + self.d:
                return 1500 + self.speed_ch_3

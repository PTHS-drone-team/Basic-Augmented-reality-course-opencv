import cv2
import numpy as np


class Rotate_drone:

    def __init__(self, radius_ch_1, radius_ch_2, speed_ch_1, speed_ch_2, dsize):
        self.radius_ch_1 = radius_ch_1
        self.radius_ch_2 = radius_ch_2
        self.dsize = dsize
        self.speed_ch_1 = speed_ch_1
        self.speed_ch_2 = speed_ch_2

    def get_center(self, marker_corners):
        self.marker_corners = marker_corners
        if len(self.marker_corners) > 0:
            self.center = (int((self.marker_corners[0][0][0][0] + self.marker_corners[0][0][2][0]) / 2),
                           int((self.marker_corners[0][0][0][1] + self.marker_corners[0][0][2][1]) / 2))
            return self.center
        else:
            return None

    def get_command_to_ch_2(self, center):
        if center is not None:
            if center[0] >= self.dsize[0] // 2 - self.radius_ch_2 and center[0] <= self.dsize[
                0] // 2 + self.radius_ch_2:
                return 1500
            elif center[0] > self.dsize[0] // 2 + self.radius_ch_2:
                return 1500 - self.speed_ch_2
            elif center[0] < self.dsize[0] // 2 - self.radius_ch_2:
                return 1500 + self.speed_ch_2
        else:
            return 1500

    def get_command_to_ch_1(self, center):
        if center is not None:
            if center[1] >= self.dsize[1] // 2 - self.radius_ch_1 and center[1] <= self.dsize[
                1] // 2 + self.radius_ch_1:
                return 1500
            elif center[1] > self.dsize[1] // 2 + self.radius_ch_1:
                return 1500 - self.speed_ch_1
            elif center[1] < self.dsize[1] // 2 - self.radius_ch_1:
                return 1500 + self.speed_ch_1
        else:
            return 1500

    def draw_line(self, frame):
        self.frame = frame
        cv2.line(self.frame, (self.dsize[0] // 2 - self.radius_ch_2, 0),
                 (self.dsize[0] // 2 - self.radius_ch_2, self.dsize[1]), (255, 0, 0))
        cv2.line(self.frame, (self.dsize[0] // 2 + self.radius_ch_2, 0),
                 (self.dsize[0] // 2 + self.radius_ch_2, self.dsize[1]), (255, 0, 0))
        cv2.line(self.frame, (0, self.dsize[1] // 2 - self.radius_ch_1),
                 (self.dsize[0], self.dsize[1] // 2 - self.radius_ch_1), (255, 0, 0))
        cv2.line(self.frame, (0, self.dsize[1] // 2 + self.radius_ch_1),
                 (self.dsize[0], self.dsize[1] // 2 + self.radius_ch_1), (255, 0, 0))

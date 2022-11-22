import cv2
from cv2 import aruco
import numpy as np
import DISTANCE_ESTIMATION.distance_class
import rotate_drone_class

calib_data_path = "calib_data_asus/MultiMatrix.npz"
MARKER_SIZE = 2.4  # размер маркера
typeAruco = aruco.DICT_4X4_50
cap = cv2.VideoCapture(0)
radius_ch_2 = 80  # расстояние до вертикальный линий от центра
radius_ch_1 = 80
speed_ch_1, speed_ch_2, speed_ch_3 = 100, 100, 100  # RC изменение
direction_change_value = 30.0  # см

Distance = DISTANCE_ESTIMATION.distance_class.DistanceAruco(calib_data_path, MARKER_SIZE, typeAruco,
                                                            direction_change_value, speed_ch_3, cap)
Rotate = rotate_drone_class.Rotate_drone(radius_ch_1, radius_ch_2, speed_ch_1, speed_ch_2, (640, 480))

while True:
    ret, frame = cap.read()

    marker_corners = Distance.get_marker_corners(frame)

    dist = Distance.get_distance(marker_corners)
    ch_3 = Distance.get_command_to_ch_3()

    center = Rotate.get_center(marker_corners)
    ch_2 = Rotate.get_command_to_ch_2(center)
    ch_1 = Rotate.get_command_to_ch_1(center)

    Rotate.draw_line(frame)

    if center is not None:
        frame = cv2.circle(frame, (center[0], center[1]), 10, (0, 255, 0), -1)
    cv2.imshow("video", frame)

    k = cv2.waitKey(1)
    if k & 0xFF == 27:  # Выход из программы, если нажали ESC
        print('esc pressed')
        cv2.destroyAllWindows()
        exit(0)

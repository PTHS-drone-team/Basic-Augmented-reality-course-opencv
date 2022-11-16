import cv2 as cv
from cv2 import aruco
import numpy as np
from pioneer_sdk import Camera, Pioneer
import time

calib_data_path = "../calib_data_pioneer/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 6  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters_create()

camera = Camera()
pioneer_mini = Pioneer()
cap = cv.VideoCapture(0)

while True:
    ch_1, ch_2, ch_3, ch_4, ch_5 = 1500, 1500, 1500, 1500, 2000
    frame_copter = camera.get_frame()
    if frame_copter is not None:
        frame = cv.imdecode(np.frombuffer(frame_copter, dtype=np.uint8), cv.IMREAD_COLOR)
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        dsize = (int(frame.shape[1]), int(frame.shape[0]))
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                print(distance)
                # Draw the pose of the marker
                #point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(
                    frame,
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right, cv.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv.LINE_AA,
                )
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right, cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2, cv.LINE_AA,
                )
                # print(distance)
                # print(ids, "  ", corners)
                if round(distance, 2) > 70.0:
                    ch_3 = 1411100
                else:
                    ch_3 = 1600

        if len(marker_corners) > 0:
            center = (int((marker_corners[0][0][0][0] + marker_corners[0][0][2][0]) / 2),
                      int((marker_corners[0][0][0][1] + marker_corners[0][0][2][1]) / 2))
            frame = cv.circle(frame, (center[0], center[1]), 10, (0, 255, 0), -1)
            r = 80
            if center[0] >= dsize[0] // 2 - r and center[0] <= dsize[0] // 2 + r:
                print("OK")
                ch_2 = 1500
            elif center[0] > dsize[0] // 2 + r:
                print("right")
                ch_2 = 1300
            elif center[0] < dsize[0] // 2 - r:
                ch_2 = 1700
                command = 1000
        cv.imshow("frame", frame)

    key = cv.waitKey(1)
    if key & 0xFF == 27:  # Выход из программы, если нажали ESC
        print('esc pressed')
        pioneer_mini.land()
        cv.destroyAllWindows()
        exit(0)
    elif key == ord('1'):
        pioneer_mini.arm()
    elif key == ord('2'):
        pioneer_mini.disarm()
    elif key == ord('3'):
        time.sleep(2)
        pioneer_mini.arm()
        time.sleep(1)
        pioneer_mini.takeoff()
        time.sleep(2)
    elif key == ord('4'):
        pioneer_mini.land()
    elif key == ord('q'):
        ch_2 = 2000
    elif key == ord('e'):
        ch_2 = 1000

    pioneer_mini.send_rc_channels(channel_1=ch_1, channel_2=ch_2, channel_3=ch_3, channel_4=ch_4,
                                      channel_5=ch_5)

cap.release()
cv.destroyAllWindows()

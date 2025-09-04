import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_id = 5
size_px = 200  # 픽셀 크기

# drawMarker(dict, id, sidePixels)
img = cv2.aruco.drawMarker(aruco_dict, marker_id, size_px)
cv2.imwrite(f'aruco_{marker_id}.png', img)

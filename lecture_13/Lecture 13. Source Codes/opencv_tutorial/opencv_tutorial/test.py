import cv2, numpy as np, os

print("OpenCV:", cv2.__version__)
DICT = cv2.aruco.DICT_4X4_50
aruco = cv2.aruco.getPredefinedDictionary(DICT)

# ▼ (A) 탐지 파라미터: Humble(4.5.x) 호환
try:
    params = cv2.aruco.DetectorParameters_create()
    use_aruco_detector_cls = False
except AttributeError:
    params = cv2.aruco.DetectorParameters()  # 4.8+
    use_aruco_detector_cls = True

# ▼ (B) 큼직하게 + 흰 여백(quiet zone) 붙여 생성
marker_id = 5
canvas = 1024
margin_ratio = 0.12       # 12% 여백
side = int(canvas*(1-2*margin_ratio))  # 실제 마커 픽셀 폭
marker = cv2.aruco.drawMarker(aruco, marker_id, side)  # borderBits 기본=1

img = np.full((canvas, canvas), 255, np.uint8)  # 흰 배경
off = (canvas-side)//2
img[off:off+side, off:off+side] = marker

# 저장
fn = "aruco_5.png"
cv2.imwrite(fn, img)
print("saved:", os.path.abspath(fn), "shape:", img.shape)

def detect(gray):
    if use_aruco_detector_cls:
        det = cv2.aruco.ArucoDetector(aruco, params)
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco, parameters=params)
    return corners, ids

# ▼ (C) 메모리에서 바로 탐지
c1, ids1 = detect(img)
print("in-memory ids:", ids1)

# ▼ (D) 파일로부터 읽어서 탐지
im2 = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
c2, ids2 = detect(im2)
print("from-file ids:", ids2)

# 디버그 보기
dbg = cv2.cvtColor(im2, cv2.COLOR_GRAY2BGR)
if ids2 is not None:
    cv2.aruco.drawDetectedMarkers(dbg, c2, ids2)
cv2.imshow("marker", dbg)
cv2.waitKey(0)

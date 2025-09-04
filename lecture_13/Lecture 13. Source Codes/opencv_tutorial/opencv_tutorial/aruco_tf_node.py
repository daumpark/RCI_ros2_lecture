#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# --------- 유틸: OpenCV 4.5.x / 4.8+ 호환 ----------
def make_aruco_detector(dict_id, params):
    aruco = cv2.aruco.getPredefinedDictionary(dict_id)
    try:
        # OpenCV 4.5.x
        _ = cv2.aruco.DetectorParameters_create
        use_cls = False
    except AttributeError:
        use_cls = True
    if use_cls:
        return aruco, cv2.aruco.ArucoDetector(aruco, params), True
    else:
        return aruco, None, False

def create_detector_params():
    try:
        return cv2.aruco.DetectorParameters_create()   # 4.5.x
    except AttributeError:
        return cv2.aruco.DetectorParameters()          # 4.8+

def rodrigues_to_quat(rvec):
    R, _ = cv2.Rodrigues(rvec)
    # 회전행렬 → 쿼터니언(x,y,z,w)
    # 수치안정 위해 trace 활용
    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        # 가장 큰 대각 원소 기준 케이스
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


class ArucoTFNode(Node):
    def __init__(self):
        super().__init__('aruco_tf_node')
        self.bridge = CvBridge()
        self.tfbr = TransformBroadcaster(self)

        # ----- 파라미터 -----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('marker_length_m', 0.12)  # 마커 한 변(m)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '~/debug_image')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.marker_len = float(self.get_parameter('marker_length_m').get_parameter_value().double_value)
        self.publish_debug_image = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        self.debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value

        # ArUco 딕셔너리 파싱
        dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        if not dict_name.startswith('DICT_'):
            dict_name = 'DICT_4X4_50'
            self.get_logger().warn('aruco_dict 파라미터가 유효하지 않아 DICT_4X4_50으로 대체합니다.')
        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)

        # Detector 준비
        params = create_detector_params()
        self.aruco, self.detector, self.use_cls = make_aruco_detector(dict_id, params)

        # 카메라 내부 파라미터
        self.K = None
        self.D = None
        self.got_info = False

        # 구독
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, 10)
        self.img_sub  = self.create_subscription(Image, self.image_topic, self.on_image, 10)

        # 디버그 이미지 퍼블리셔(옵션)
        if self.publish_debug_image:
            from rclpy.qos import QoSProfile
            self.debug_pub = self.create_publisher(Image, self.debug_image_topic, QoSProfile(depth=1))

        self.get_logger().info(f"Subscribing: image={self.image_topic}, info={self.camera_info_topic}")
        self.get_logger().info(f"Broadcasting TF: {self.camera_frame} -> aruco_<id>, marker={self.marker_len} m, dict={dict_name}")

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        # D 길이는 카메라에 따라 다름(5 또는 그 이상)
        self.D = np.array(msg.d, dtype=np.float64).ravel()
        self.got_info = True

    def on_image(self, msg: Image):
        # 0) 입력 상태 로그 (1초에 한 번 정도 보고 싶으면 throttle로 바꿔도 됨)
        self.get_logger().debug(f"image encoding={msg.encoding} size=({msg.width}x{msg.height}) frame_id='{msg.header.frame_id}'")

        # 1) 이미지 가져오기
        try:
            cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if cvimg.ndim == 3:
                pass
            else:
                cvimg = cv2.cvtColor(cvimg, cv2.COLOR_GRAY2BGR)
        gray = cv2.cvtColor(cvimg, cv2.COLOR_BGR2GRAY)
        dbg = cvimg.copy()

        # (옵션) 작은 마커라면 업스케일 한번
        if min(gray.shape[:2]) < 600:
            scale = 2.0
            gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            dbg  = cv2.resize(dbg,  None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

        # 2) 대비 향상 + 파라미터 완화
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray_eq = clahe.apply(gray)

        # detector params (보수→완화)
        try:
            p = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            p = cv2.aruco.DetectorParameters()
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        p.adaptiveThreshWinSizeMin = 3
        p.adaptiveThreshWinSizeMax = 53
        p.adaptiveThreshWinSizeStep = 4
        p.adaptiveThreshConstant = 7
        p.minMarkerPerimeterRate = 0.02   # 기본 0.03; 더 작게
        p.maxMarkerPerimeterRate = 4.0
        p.minCornerDistanceRate = 0.02
        p.minDistanceToBorder = 1
        p.polygonalApproxAccuracyRate = 0.03

        # 3) 탐지 (클래스/함수 호환)
        if self.use_cls:
            det = cv2.aruco.ArucoDetector(self.aruco, p)
            corners, ids, _ = det.detectMarkers(gray_eq)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray_eq, self.aruco, parameters=p)

        # 4) 시각화 (탐지만이라도 즉시 확인)
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
            self.get_logger().info(f"DETECTED ids={ids.ravel().tolist()}")
        else:
            self.get_logger().debug("No markers detected in this frame.")

        # 5) pose & TF (CameraInfo 있을 때만)
        if ids is not None and len(ids) > 0 and self.got_info:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_len, self.K, self.D)
            parent_frame = self.camera_frame if self.camera_frame else (msg.header.frame_id or "camera")
            for (marker_id, rvec, tvec) in zip(ids.ravel().tolist(), rvecs, tvecs):
                # axis for debug
                cv2.aruco.drawAxis(dbg, self.K, self.D, rvec, tvec, self.marker_len * 0.5)
                tfm = TransformStamped()
                tfm.header.stamp = msg.header.stamp
                tfm.header.frame_id = parent_frame
                tfm.child_frame_id = f"aruco_{marker_id}"
                t = tvec.reshape(-1)
                tfm.transform.translation.x = float(t[0])
                tfm.transform.translation.y = float(t[1])
                tfm.transform.translation.z = float(t[2])
                qx, qy, qz, qw = rodrigues_to_quat(rvec.reshape(-1))
                tfm.transform.rotation.x = qx; tfm.transform.rotation.y = qy
                tfm.transform.rotation.z = qz; tfm.transform.rotation.w = qw
                self.tfbr.sendTransform(tfm)

        # 6) 디스플레이/퍼블리시
        if self.publish_debug_image:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8'))
            cv2.imshow("aruco_debug", dbg)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = ArucoTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

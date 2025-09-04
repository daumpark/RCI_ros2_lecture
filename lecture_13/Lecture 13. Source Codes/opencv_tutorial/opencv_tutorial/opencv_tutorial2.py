#!/usr/bin/env python3
import cv2
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros

class RedPillarPresenceTF(Node):
    def __init__(self):
        super().__init__('red_pillar_presence_tf')

        # ---- Parameters ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('show_windows', True)
        self.declare_parameter('min_area', 300)                # 작은 노이즈 제거
        self.declare_parameter('min_bbox_h_px', 8)             # bbox 최소 높이 (너무 작으면 무시)
        self.declare_parameter('erode', 1)
        self.declare_parameter('dilate', 2)
        self.declare_parameter('pillar_height_m', 0.5)         # [m] 기둥 실제 높이
        self.declare_parameter('pillar_frame', 'red_pillar')
        self.declare_parameter('parent_frame', 'base_link')     # TF 부모 프레임 (로봇 기준)
        # 카메라 내참수 fallback (CameraInfo가 없을 때 사용)
        self.declare_parameter('hfov_deg', 90.0)               # 수평 FOV
        self.declare_parameter('vfov_deg', 60.0)               # 수직 FOV

        # HSV 두 구간 (빨강)
        self.declare_parameter('hsv1_low',  [0,   120, 80])
        self.declare_parameter('hsv1_high', [10,  255, 255])
        self.declare_parameter('hsv2_low',  [170, 120, 80])
        self.declare_parameter('hsv2_high', [180, 255, 255])

        topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.show = bool(self.get_parameter('show_windows').value)

        self.bridge = CvBridge()
        self.sub_img  = self.create_subscription(Image, topic, self.on_image, 10)
        self.sub_info = self.create_subscription(CameraInfo, info_topic, self.on_caminfo, 10)
        self.pub_exists = self.create_publisher(Bool, 'red_pillar/exists', 10)
        self.pub_text   = self.create_publisher(String, 'red_pillar/status_text', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_state = None
        self.cam_K = None   # fx, fy, cx, cy를 담기 위해 K를 저장
        self.img_size = None  # (w, h)

        self.get_logger().info(f'Listening image: {topic}')
        self.get_logger().info(f'Listening camera_info: {info_topic}')

    def on_caminfo(self, msg: CameraInfo):
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.cam_K = msg.k
        self.img_size = (msg.width, msg.height)

    def _get_intrinsics(self, w: int, h: int):
        """
        fx, fy, cx, cy 반환.
        - CameraInfo가 있으면 그대로 사용
        - 없으면 FOV로부터 근사: fx = (w/2)/tan(hfov/2), fy = (h/2)/tan(vfov/2)
        """
        if self.cam_K is not None:
            fx = self.cam_K[0]; fy = self.cam_K[4]
            cx = self.cam_K[2]; cy = self.cam_K[5]
            return fx, fy, cx, cy

        # Fallback: FOV 기반 근사
        hfov = math.radians(float(self.get_parameter('hfov_deg').value))
        vfov = math.radians(float(self.get_parameter('vfov_deg').value))
        fx = (w * 0.5) / math.tan(hfov * 0.5)
        fy = (h * 0.5) / math.tan(vfov * 0.5)
        cx = w * 0.5
        cy = h * 0.5
        return fx, fy, cx, cy

    def on_image(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h_img, w_img = bgr.shape[:2]
        fx, fy, cx, cy = self._get_intrinsics(w_img, h_img)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        p = lambda n: np.array(self.get_parameter(n).value, dtype=np.uint8)
        mask1 = cv2.inRange(hsv, p('hsv1_low'), p('hsv1_high'))
        mask2 = cv2.inRange(hsv, p('hsv2_low'), p('hsv2_high'))
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphology
        k = np.ones((3,3), np.uint8)
        e_it = int(self.get_parameter('erode').value)
        d_it = int(self.get_parameter('dilate').value)
        if e_it > 0: mask = cv2.erode(mask, k, iterations=e_it)
        if d_it > 0: mask = cv2.dilate(mask, k, iterations=d_it)

        min_area = float(self.get_parameter('min_area').value)
        min_bbox_h = int(self.get_parameter('min_bbox_h_px').value)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 가장 큰 컨투어 선택 (기둥 후보)
        best = None
        best_area = 0.0
        for c in contours:
            a = cv2.contourArea(c)
            if a >= min_area and a > best_area:
                best = c
                best_area = a

        exists = best is not None

        # 상태 변화 시 텍스트 로그
        if self.last_state is None or exists != self.last_state:
            text = 'RED PILLAR: PRESENT' if exists else 'RED PILLAR: ABSENT'
            self.get_logger().info(text)
            self.pub_text.publish(String(data=text))
            self.last_state = exists

        # 결과 퍼블리시 (항상)
        self.pub_exists.publish(Bool(data=exists))

        # ---- 거리/방위 추정 & TF 브로드캐스트 ----
        if exists:
            x_tf = y_tf = z_tf = None
            bx, by, bw, bh = cv2.boundingRect(best)

            if bh >= min_bbox_h:
                H = float(self.get_parameter('pillar_height_m').value)   # 실제 기둥 높이 [m]
                # 핀홀 모델: z ≈ H * fy / h_pix
                z = (H * fy) / float(bh)

                # x(화면-가로 중심) → 좌우 오프셋
                u = bx + bw * 0.5
                x_cam = (u - cx) * (z / fx)     # 카메라 좌표계: x=오른쪽(+), z=앞(+)
                # 여기서는 로봇 평면에서 y를 좌측(+)로 쓰는 base_link를 가정
                x_tf = z                        # base_link x: 전방
                y_tf = -x_cam                   # 카메라 오른쪽(+) => base_link 좌표 y는 왼쪽(+), 부호 뒤집음
                z_tf = 0.0

                # 품질 텍스트
                angle_rad = math.atan2(y_tf, x_tf)
                angle_deg = math.degrees(angle_rad)
                info = f'range={z:.2f} m, bearing={angle_deg:.1f} deg, bbox_h={bh}px'
                self.pub_text.publish(String(data=info))
                self.get_logger().info(info)

                # TF publish
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = self.get_parameter('parent_frame').value
                t.child_frame_id = self.get_parameter('pillar_frame').value
                t.transform.translation.x = float(x_tf)
                t.transform.translation.y = float(y_tf)
                t.transform.translation.z = float(z_tf)
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                # Z축만 간단히 회전(기둥 방향을 바라보는 yaw)
                half_yaw = angle_rad * 0.5
                t.transform.rotation.z = math.sin(half_yaw)
                t.transform.rotation.w = math.cos(half_yaw)
                self.tf_broadcaster.sendTransform(t)

        # ---- 디버깅 뷰 ----
        if self.show:
            dbg = bgr.copy()
            cv2.drawContours(dbg, contours, -1, (0,255,0), 2)
            if exists:
                cv2.rectangle(dbg, (bx,by), (bx+bw,by+bh), (255,0,0), 2)
                cv2.putText(dbg, f"h={bh}px", (bx, by-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
            cv2.imshow('red_pillar_debug', dbg)
            cv2.imshow('red_mask', mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                self.show = False

def main():
    rclpy.init()
    node = RedPillarPresenceTF()
    try:
        rclpy.spin(node)
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

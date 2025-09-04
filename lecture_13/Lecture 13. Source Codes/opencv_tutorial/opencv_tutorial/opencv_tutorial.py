#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros

class RedPillarPresenceTF(Node):
    def __init__(self):
        super().__init__('red_pillar_presence_tf')

        # ---- 파라미터 ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('show_windows', True)
        self.declare_parameter('min_area', 300)          # 너무 작은 노이즈 무시
        self.declare_parameter('erode', 1)
        self.declare_parameter('dilate', 2)
        self.declare_parameter('pillar_distance', 1.0)   # m, 기둥까지 거리 가정
        self.declare_parameter('pillar_frame', 'red_pillar')

        # HSV 두 구간 (빨강)
        self.declare_parameter('hsv1_low',  [0,   120, 80])
        self.declare_parameter('hsv1_high', [10,  255, 255])
        self.declare_parameter('hsv2_low',  [170, 120, 80])
        self.declare_parameter('hsv2_high', [180, 255, 255])

        topic = self.get_parameter('image_topic').value
        self.show = bool(self.get_parameter('show_windows').value)

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.on_image, 10)
        self.pub_exists = self.create_publisher(Bool, 'red_pillar/exists', 10)
        self.pub_text   = self.create_publisher(String, 'red_pillar/status_text', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_state = None
        self.get_logger().info(f'Listening: {topic}')

    def on_image(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        p = lambda n: np.array(self.get_parameter(n).value, dtype=np.uint8)
        mask1 = cv2.inRange(hsv, p('hsv1_low'), p('hsv1_high'))
        mask2 = cv2.inRange(hsv, p('hsv2_low'), p('hsv2_high'))
        mask = cv2.bitwise_or(mask1, mask2)

        # 모폴로지 정리
        k = np.ones((3,3), np.uint8)
        e_it = int(self.get_parameter('erode').value)
        d_it = int(self.get_parameter('dilate').value)
        if e_it > 0: mask = cv2.erode(mask, k, iterations=e_it)
        if d_it > 0: mask = cv2.dilate(mask, k, iterations=d_it)

        # 충분히 큰 빨강 영역이 있는지 확인
        min_area = float(self.get_parameter('min_area').value)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        exists = any(cv2.contourArea(c) >= min_area for c in contours)

        # 상태 변화 시만 로그 & 텍스트 퍼블리시
        if self.last_state is None or exists != self.last_state:
            text = 'RED PILLAR: PRESENT' if exists else 'RED PILLAR: ABSENT'
            self.get_logger().info(text)
            self.pub_text.publish(String(data=text))
            self.last_state = exists

        # 결과 퍼블리시 (항상)
        self.pub_exists.publish(Bool(data=exists))

        # ---- TF 브로드캐스트 ----
        if exists:
            dist = float(self.get_parameter('pillar_distance').value)
            pillar_frame = self.get_parameter('pillar_frame').value

            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = pillar_frame
            t.transform.translation.x = dist
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)

        # 디버깅 표시
        if self.show:
            dbg = bgr.copy()
            cv2.drawContours(dbg, contours, -1, (0,255,0), 2)
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


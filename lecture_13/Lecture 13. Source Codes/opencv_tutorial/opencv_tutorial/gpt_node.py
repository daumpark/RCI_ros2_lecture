#!/usr/bin/env python3
# cola_can_detector_gpt.py
import os
import time
import base64
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# OpenAI SDK (pip install openai)
from openai import OpenAI

INSTRUCTION = (
    "You are a strict visual classifier. "
    "Answer ONLY with a single JSON object like: "
    '{"present": true, "confidence": 0.0, "reason": "…"} . '
    'Definition: "present" means a red cola can (Coca-Cola style) is clearly visible. '
    "If unsure, set present=false. Confidence in [0,1]. Keep 'reason' short."
)

class ColaCanDetectorGPT(Node):
    def __init__(self):
        super().__init__("cola_can_detector_gpt")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("model", "gpt-4o")  # 비전 입력 지원 모델
        self.declare_parameter("interval_sec", 2.0)  # 과금/속도 제어용 샘플링 간격
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("resize_width", 640)  # 전송량 줄이기 (0이면 원본)
        self.declare_parameter("timeout_sec", 20.0)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.interval_sec = float(self.get_parameter("interval_sec").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.resize_width = int(self.get_parameter("resize_width").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.client = OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
        self.bridge = CvBridge()
        self.last_query_time = 0.0

        # Publishers
        self.pub_present = self.create_publisher(Bool, "/cola_can/present", 10)
        self.pub_conf = self.create_publisher(Float32, "/cola_can/confidence", 10)
        self.pub_reason = self.create_publisher(String, "/cola_can/reason", 10)
        self.pub_raw = self.create_publisher(String, "/cola_can/raw", 10)  # 원문 JSON

        # Subscriber
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)

        self.get_logger().info(f"cola_can_detector_gpt started. Subscribing: {image_topic}, model: {self.model}")

    def image_cb(self, msg: Image):
        now = time.time()
        if now - self.last_query_time < self.interval_sec:
            return  # 샘플링 간격 유지
        self.last_query_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}")
            return

        # 선택적 리사이즈 (전송량/비용 절감)
        if self.resize_width > 0 and cv_img.shape[1] > self.resize_width:
            h, w = cv_img.shape[:2]
            new_w = self.resize_width
            new_h = int(h * (new_w / w))
            cv_img = cv2.resize(cv_img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # JPEG로 인코딩
        ok, buf = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warn("JPEG encode failed.")
            return

        b64 = base64.b64encode(buf.tobytes()).decode("ascii")
        data_url = f"data:image/jpeg;base64,{b64}"

        try:
            # Responses API로 비전 프롬프트 전송
            resp = self.client.responses.create(
    model=self.model,
    input=[{
        "role": "user",
        "content": [
            {"type": "input_text", "text": INSTRUCTION},
            {"type": "input_image", "image_url": data_url}  # ← 문자열로!
        ]
    }],
    temperature=0
)
            # 편의 속성(지원되는 SDK 버전에서): 전체 텍스트
            text = getattr(resp, "output_text", None)
            if not text:
                # 구조적으로 꺼내기(버전에 따라 달라질 수 있어 폴백 제공)
                try:
                    text = resp.output[0].content[0].text
                except Exception:
                    text = str(resp)

            # 기대 포맷: {"present": true/false, "confidence": 0.xx, "reason":"..."}
            present, conf, reason = self.parse_json_like(text)

            # Publish
            self.pub_present.publish(Bool(data=present))
            self.pub_conf.publish(Float32(data=conf))
            self.pub_reason.publish(String(data=reason))
            self.pub_raw.publish(String(data=text))

            self.get_logger().info(f"present={present} conf={conf:.2f} reason={reason}")

        except Exception as e:
            self.get_logger().error(f"OpenAI call failed: {e}")

    @staticmethod
    def parse_json_like(text: str):
        import json, re
        # JSON 그대로면 파싱, 아니면 간단 추정
        try:
            obj = json.loads(text)
            present = bool(obj.get("present", False))
            conf = float(obj.get("confidence", 0.0))
            reason = str(obj.get("reason", ""))
            conf = max(0.0, min(1.0, conf))
            return present, conf, reason
        except Exception:
            # yes/no류 방어코드
            low = text.strip().lower()
            if "true" in low or "yes" in low:
                return True, 0.5, "non-json yes/true"
            return False, 0.5, "non-json no/false"

def main():
    rclpy.init()
    node = ColaCanDetectorGPT()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

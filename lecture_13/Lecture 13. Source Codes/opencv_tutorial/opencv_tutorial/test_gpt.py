#!/usr/bin/env python3
import os, io, time, json, base64
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

# OpenAI SDK v1
from openai import OpenAI
client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

PROMPT = (
    "You are a vision inspector in a Gazebo TurtleBot3 world. "
    "Look only for a typical red soda can (e.g., Coca-Cola style). "
    "Answer in strict JSON with keys: present (bool), reason (string). "
    "If unsure, set present=false. No extra text."
)

def encode_jpeg_b64(bgr, quality=85):
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return base64.b64encode(buf.tobytes()).decode("ascii")

def has_enough_red_hint(bgr, min_ratio=0.002):
    # (A) HSV 마스크 (완화된 임계값)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 50, 40),  (12, 255, 255))
    m2 = cv2.inRange(hsv, (170, 50, 40),(180, 255, 255))
    hsv_mask = cv2.bitwise_or(m1, m2)

    # (B) 단순 BGR 규칙: R이 G/B보다 충분히 크고 R 자체도 어느 정도 밝기
    B, G, R = cv2.split(bgr)
    bgr_rule = (R > 90) & (R > 1.2 * G) & (R > 1.2 * B)
    bgr_mask = bgr_rule.astype(np.uint8) * 255

    # (C) 합치고 약간 팽창해서 소실 방지
    mask = cv2.bitwise_or(hsv_mask, bgr_mask)
    mask = cv2.dilate(mask, None, iterations=1)

    ratio = float(np.count_nonzero(mask)) / (mask.size + 1e-9)
    return ratio >= min_ratio, ratio

def call_gpt_vision(jpeg_b64, max_retries=5):
    for i in range(max_retries):
        try:
            resp = client.responses.create(
                model="gpt-4o-mini",
                input=[{
                    "role": "user",
                    "content": [
                        {"type": "input_text", "text": PROMPT},
                        # ⬇️ 여기! image → image_url (문자열) 로 변경
                        {"type": "input_image",
                         "image_url": f"data:image/jpeg;base64,{jpeg_b64}"}
                    ]
                }],
                # SDK 최신이면 사용, 구버전이면 이 줄은 지우세요
                # response_format={"type": "json_object"}
            )
            out = getattr(resp, "output_text", None) or ""
            return json.loads(out) if out.strip().startswith("{") else {"present": False, "reason": out}
        except Exception as e:
            if "429" in str(e).lower() or "insufficient_quota" in str(e).lower():
                time.sleep(0.8 * (2 ** i))
                continue
            raise
            
            out = resp.output_text  # 또는 resp.output[0].content[0].text
            result = json.loads(out) if out.strip().startswith("{") else {"present": False, "reason": out}
            if out is None and hasattr(resp, "output") and len(resp.output)>0 and hasattr(resp.output[0], "content"):
                # 안전망
                out = "".join(getattr(c, "text", "") for c in resp.output[0].content if hasattr(c, "text"))
            if not out:
                raise RuntimeError("Empty response from model")
            try:
                return json.loads(out)
            except json.JSONDecodeError:
                # 혹시 JSON 이외가 내려오면 대충 파싱
                lowered = out.strip().lower()
                present = "true" in lowered and "false" not in lowered
                return {"present": bool(present), "reason": out[:120]}
        except Exception as e:
            msg = str(e)
            # 429/쿼터/레이트리밋
            if "429" in msg or "insufficient_quota" in msg or "rate limit" in msg.lower():
                time.sleep(0.8 * (2 ** i))
                continue
            # 기타 에러는 바로 전달
            raise

class ColaCanGPT(Node):
    def __init__(self):
        super().__init__("cola_can_detector_gpt")

        # 파라미터
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("show_debug", False)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("prefilter_min_red_ratio", 0.0)  # 0.2%
        self.declare_parameter("call_every_n_frames", 3)          # 프레임 스로틀
        self.declare_parameter("model", "gpt-4o-mini")

        self.bridge = CvBridge()
        self.img_topic = self.get_parameter("image_topic").value
        self.show_debug = self.get_parameter("show_debug").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.min_red_ratio = float(self.get_parameter("prefilter_min_red_ratio").value)
        self.call_every_n = int(self.get_parameter("call_every_n_frames").value)
        self.model = self.get_parameter("model").value

        self.pub_present = self.create_publisher(Bool, "/cola_can/present", 10)
        self.pub_reason  = self.create_publisher(String, "/cola_can/reason", 10)
        if self.show_debug:
            self.pub_dbg = self.create_publisher(Image, "/cola_can/debug", 10)
        else:
            self.pub_dbg = None

        self.frame_count = 0
        self.sub = self.create_subscription(Image, self.img_topic, self.on_image, 10)
        self.get_logger().info(f"Subscribing {self.img_topic} / model={self.model}")

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        self.frame_count += 1
        # 토큰/요금 절감: N프레임마다만 호출
        if self.frame_count % self.call_every_n != 0:
            return

        # 빨강 프리필터
        ok, ratio = has_enough_red_hint(bgr, self.min_red_ratio)
        if not ok:
            self.publish(False, f"red-prefilter ratio={ratio:.4f}")
            if self.pub_dbg:
                self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8"))
            return

        # JPEG 인코딩 → base64
        try:
            jpeg_b64 = encode_jpeg_b64(bgr, quality=self.jpeg_quality)
        except Exception as e:
            self.get_logger().warn(f"JPEG encode failed: {e}")
            return

        # GPT 호출
        try:
            result = call_gpt_vision(jpeg_b64)
            present = bool(result.get("present", False))
            reason  = str(result.get("reason", ""))
            self.publish(present, reason if reason else "no reason")
        except Exception as e:
            # 429/쿼터 등 오류 메세지를 그대로 reason에 녹여서 디버그에 도움
            self.publish(False, f"gpt_error: {e}")

        # 디버그 이미지 퍼블리시(선택)
        if self.pub_dbg:
            vis = bgr.copy()
            # 아주 러프하게 빨강 영역만 표시
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            m1 = cv2.inRange(hsv, (0,120,80), (10,255,255))
            m2 = cv2.inRange(hsv, (170,120,80), (180,255,255))
            mask = cv2.bitwise_or(m1, m2)
            vis[mask>0] = (0,0,255)
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(vis, encoding="bgr8"))

    def publish(self, present: bool, reason: str):
        self.pub_present.publish(Bool(data=present))
        self.pub_reason.publish(String(data=reason))
        self.get_logger().info(f"present={present} | {reason}")

def main():
    rclpy.init()
    node = ColaCanGPT()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


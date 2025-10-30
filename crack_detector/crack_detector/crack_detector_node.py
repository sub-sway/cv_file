#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
import pymongo
import base64
from datetime import datetime, timezone
import time, pytz

# ⭐️ 추가된 임포트
from sensor_msgs.msg import Image       # ROS2 이미지 메시지
from cv_bridge import CvBridge, CvBridgeError # ROS <-> OpenCV 이미지 변환

class CrackDetectorNode(Node):
    """
    'image_raw' 토픽을 구독하여 이미지를 수신하고, 균열을 감지하여
    일정 간격으로 MongoDB에 데이터를 저장하는 ROS 2 노드입니다.
    """
    def __init__(self):
        super().__init__('crack_detector_node')
        self.get_logger().info("🚀 Crack Detection Subscriber Node Started (Cooldown Enabled)")

        # --- 설정 값 ---
        self.device_name = "jetson_orin_01"
        self.cooldown_seconds = 30.0
        self.model_path = "/home/orin/ros2_ws/src/crack_detector/crack_detector/Crack4.engine"
        self.confidence_threshold = 0.65
        self.seoul_timezone = pytz.timezone("Asia/Seoul")
        
        # --- 내부 변수 ---
        self.last_sent_time = 0.0

        # --- YOLO 모델 로드 ---
        try:
            self.model = YOLO(self.model_path, task="detect")
            self.get_logger().info(f"✅ Model loaded: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            raise SystemExit

        # --- MongoDB Atlas 연결 ---
        try:
            mongo_uri = "mongodb+srv://jystarwow_db_user:zf01VaAW4jYH0dVP@porty.oqiwzud.mongodb.net/"
            self.mongo_client = pymongo.MongoClient(mongo_uri)
            self.db = self.mongo_client["crack_monitor"]
            self.collection = self.db["crack_results"]
            self.mongo_client.admin.command('ping')
            self.get_logger().info("✅ MongoDB Atlas connected successfully.")
        except Exception as e:
            self.get_logger().error(f"❌ Could not connect to MongoDB Atlas: {e}")
            raise SystemExit
            
        # ⭐️ CV 브릿지 초기화
        self.bridge = CvBridge()
        
        # ⭐️ 타이머 대신 구독자 생성
        # '/image_raw' 토픽을 구독하고, 메시지가 오면 self.image_callback 함수를 호출합니다.

        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10) # QoS 프로파일, 10은 버퍼 크기
        self.get_logger().info("👂 Subscribed to /image_raw topic. Waiting for images...")
        
        cv2.namedWindow("Live Detection", cv2.WINDOW_NORMAL)

    # ⭐️ 새롭게 추가된 콜백 함수
    def image_callback(self, msg):
        """
        이미지 메시지를 수신하면 호출되어 균열 탐지를 수행합니다.
        """
        try:
            # ROS Image 메시지를 OpenCV 이미지(numpy array)로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return
            
        # --- 이하 탐지 로직은 기존 detect_and_save 함수와 거의 동일 ---
        results = self.model(frame, verbose=False)
        r = results[0]

        high_conf_detections = []
        if r.boxes:
            for box in r.boxes:
                if box.conf[0] >= self.confidence_threshold:
                    high_conf_detections.append({
                        "class_name": self.model.names[int(box.cls[0])],
                        "confidence": float(box.conf[0]),
                        "box_xyxy": [float(coord) for coord in box.xyxy[0]]
                    })

        if high_conf_detections:
            now = time.time()
            if (now - self.last_sent_time) > self.cooldown_seconds:
                self.last_sent_time = now
                self.get_logger().info(f"✅ Detected {len(high_conf_detections)} crack(s). Cooldown timer reset. Saving to DB...")
                self.save_to_db(r, high_conf_detections)
            else:
                remaining_time = self.cooldown_seconds - (now - self.last_sent_time)
                self.get_logger().info(f"⏱️ Crack detected, but in cooldown period. Skipping send. ({remaining_time:.1f}s left)")

        display_frame = r.plot() if high_conf_detections else frame
        cv2.imshow("Live Detection", display_frame)
        cv2.waitKey(1)

    def save_to_db(self, r, detections):
        annotated_frame = r.plot()
        _, buffer = cv2.imencode(".jpg", annotated_frame)
        b64_img = base64.b64encode(buffer).decode('utf-8')

        detection_log = {
            "timestamp": datetime.now(self.seoul_timezone),
            "num_detections": len(detections),
            "detections": detections,
            "annotated_image_base64": b64_img
        }
        
        try:
            self.collection.insert_one(detection_log)
            self.get_logger().info("💾 Successfully saved to MongoDB.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save to MongoDB: {e}")

    def destroy_node(self):
        # ⭐️ 카메라 해제(cap.release) 코드는 더 이상 필요 없으므로 제거
        cv2.destroyAllWindows()
        self.get_logger().info("🛑 Node Shutting Down")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CrackDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 User requested shutdown (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

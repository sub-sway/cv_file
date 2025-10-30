#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2, time, json, socket, subprocess, pkg_resources, ssl, random, os, threading, base64
import paho.mqtt.client as mqtt
from ultralytics import YOLO
import pygame, pymongo
from pymongo.errors import ConnectionFailure
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from datetime import datetime

# ==========================
# numpy 버전 자동 조정 (이전과 동일)
# ...
# ==========================
#required_version = '1.23.1'
#try:
#    numpy_version = pkg_resources.get_distribution("numpy").version
#    if numpy_version != required_version:
#        print(f"[INFO] numpy {numpy_version} → {required_version}로 교체 중...")
#        subprocess.run(["pip", "install", f"numpy=={required_version}", "--force-reinstall"], check=True)
#except Exception as e:
 #   print(f"[WARN] numpy 버전 확인 실패: {e}")

# ==========================
# 헬퍼 함수들 (이전과 동일)
# ...
# ==========================
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = "127.0.0.1"
    return ip

def play_sound(alert_type):
    try:
        sound_dir = "/home/orin/ros2_ws/src/yolo_alert_pkg/yolo_alert_pkg/sounds"
        sound_file = ""
        if alert_type == "fire":
            sound_file = os.path.join(sound_dir, "fire_cut.wav")
        elif alert_type == "safety":
            sound_file = os.path.join(sound_dir, "Stranger_cut.wav")
        else: return
        if not os.path.exists(sound_file):
            print(f"⚠️ 사운드 파일을 찾을 수 없습니다: {sound_file}")
            return
        pygame.mixer.music.load(sound_file)
        pygame.mixer.music.play()
    except Exception as e:
        print(f"⚠️ 사운드 재생 실패: {e}")

# ==========================
# YOLO + MQTT + MongoDB ROS2 노드
# ==========================
class YoloAlertNode(Node):
    def __init__(self):
        super().__init__('yolo_alert_node')
        self.get_logger().info("🚀 [구독자 모드] 모든 기능이 통합된 감지 노드를 시작합니다. 🚀")
        
        self.device_name = "jetson_orin_01"

        # ... pygame, MQTT, MongoDB, YOLO 모델 로드 등 이전 코드는 모두 동일 ...
        try: pygame.mixer.init()
        except Exception as e: self.get_logger().error(f"❌ Pygame Mixer 초기화 실패: {e}")
        self.broker = "porty1-3d89cd05.a01.euc1.aws.hivemq.cloud"
        self.port = 8884
        self.username = "JetsonOrin"
        self.password = "One24511"
        self.topic = "robot/alerts"
        client_id = f"ros2-node-{random.randint(0,1000)}"
        self.client = mqtt.Client(client_id=client_id, transport="websockets")
        self.client.username_pw_set(self.username, self.password)
        self.client.tls_set(cert_reqs=ssl.CERT_NONE)
        try:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            self.get_logger().info("✅ HiveMQ Cloud 연결 성공")
        except Exception as e: self.get_logger().error(f"❌ MQTT 연결 실패: {e}")
        self.mongo_client = None
        try:
            mongo_uri = "mongodb+srv://jystarwow_db_user:zf01VaAW4jYH0dVP@porty.oqiwzud.mongodb.net/?retryWrites=true&w=majority&appName=PORTY"
            self.mongo_client = pymongo.MongoClient(mongo_uri)
            self.mongo_client.admin.command('ping')
            self.db = self.mongo_client["HIvisDB"]
            self.collection = self.db["HivisData"]
            self.get_logger().info(f"✅ MongoDB Atlas 연결 성공")
        except ConnectionFailure as e:
            self.get_logger().error(f"❌ MongoDB Atlas 연결 실패: {e}")
            self.mongo_client = None
        try:
            self.fire_model = YOLO('/home/orin/ros2_ws/src/yolo_alert_pkg/yolo_alert_pkg/models/fire_s.engine', task='detect')
            self.hivis_model = YOLO('/home/orin/ros2_ws/src/yolo_alert_pkg/yolo_alert_pkg/models/hi_vis.engine', task='detect')
            self.get_logger().info("🔥 YOLO 모델 로드 완료")
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise

        # --- ROS 설정: 이미지 구독자 ---

        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.get_logger().info("👂 'image_raw' 토픽 구독 시작. 이미지 수신 대기 중...")
        
        # ✅ [추가됨] 1. 라이브 스트림을 위한 OpenCV 창 생성
        cv2.namedWindow("Live Detection", cv2.WINDOW_NORMAL)
        self.get_logger().info("🖥️ 라이브 스트림 창이 활성화되었습니다.")
        
        self.last_alert_time = {"fire": 0, "safety": 0, "normal": 0}
        self.alert_cooldowns = {"fire": 10.0, "safety": 10.0, "normal": 60.0}

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge 변환 실패: {e}")
            return
        
        current_time = time.time()
        alert_triggered_this_cycle = False
        
        # ✅ [수정됨] 화면에 표시할 최종 이미지를 담을 변수
        display_frame = frame.copy() 

        # 화재 감지 로직
        if current_time - self.last_alert_time["fire"] > self.alert_cooldowns["fire"]:
            fire_res = self.fire_model(frame, verbose=False)
            for box in fire_res[0].boxes:
                conf = float(box.conf)
                if self.fire_model.names[int(box.cls)] == "fire" and conf > 0.7:
                    self.send_alert("fire", {"message": f"🔥 화재 감지 (신뢰도 {conf:.2f})"})
                    play_sound("fire")
                    self.last_alert_time["fire"] = current_time
                    alert_triggered_this_cycle = True
                    display_frame = fire_res[0].plot() # 화재 감지 시 결과 이미지로 교체
                    break
        
        # 안전조끼 미착용 감지
        if not alert_triggered_this_cycle and \
           current_time - self.last_alert_time["safety"] > self.alert_cooldowns["safety"]:
            CONF_THRESHOLD = 0.7
            hivis_res = self.hivis_model(frame, verbose=False)
            high_conf_names = [self.hivis_model.names[int(box.cls)] for box in hivis_res[0].boxes if float(box.conf) > CONF_THRESHOLD]
            
            if "person" in high_conf_names and "hi-vis" not in high_conf_names:
                details = {"person_count": high_conf_names.count("person"), "hivis_count": high_conf_names.count("hi-vis")}
                self.send_alert("safety", {"message": "⚠️ 안전조끼 미착용 인원 발견", **details})
                play_sound("safety")
                self.save_detection_to_db(frame, hivis_res)
                self.last_alert_time["safety"] = current_time
                alert_triggered_this_cycle = True
                display_frame = hivis_res[0].plot() # 안전조끼 미착용 감지 시 결과 이미지로 교체

        # 정상 상태 알림
        if not alert_triggered_this_cycle and \
           current_time - self.last_alert_time["normal"] > self.alert_cooldowns["normal"]:
            self.send_alert("normal", {"message": "✅ 시스템 정상 작동 중"})
            self.last_alert_time["normal"] = current_time
        
        # ✅ [추가됨] 2. 최종 이미지를 화면에 표시하고 GUI를 업데이트
        cv2.imshow("Live Detection", display_frame)
        cv2.waitKey(1)

    # --- send_alert, save_detection_to_db 함수는 이전과 동일 ---
    def send_alert(self, alert_type, payload):
        data = { "type": alert_type, "payload": payload, "timestamp": datetime.now().isoformat(), "source_ip": get_local_ip() }
        try:
            self.client.publish(self.topic, json.dumps(data))
            if alert_type in ["fire", "safety"]: self.get_logger().warn(f"[MQTT 발행] {alert_type.upper()} Event")
        except Exception as e: self.get_logger().error(f"MQTT 발행 실패: {e}")

    def save_detection_to_db(self, frame, detection_results):
        if not self.mongo_client: return
        detections_list = [{"class_name": self.hivis_model.names[int(box.cls)], "confidence": float(box.conf), "box_xyxy": box.xyxy[0].tolist()} for box in detection_results[0].boxes]
        annotated_frame = detection_results[0].plot()
        _, buffer = cv2.imencode('.jpg', annotated_frame)
        annotated_image_base64 = base64.b64encode(buffer).decode('utf-8')
        log_data = {"timestamp": datetime.now(), "source_device": self.device_name, "detections": detections_list, "annotated_image_base64": annotated_image_base64}
        try:
            self.collection.insert_one(log_data)
            self.get_logger().info(f"[DB 저장] 안전조끼 미착용 이벤트 저장 완료")
        except Exception as e: self.get_logger().error(f"MongoDB 저장 실패: {e}")

    # ✅ [추가됨] 3. 노드 종료 시 호출될 함수에 창 닫기 기능 추가
    def destroy_node(self):
        self.get_logger().info("🛑 종료 중... 리소스를 정리합니다.")
        cv2.destroyAllWindows() # 열려있는 모든 OpenCV 창 닫기
        if hasattr(self, 'client') and self.client.is_connected():
            self.client.loop_stop()
            self.client.disconnect()
        if hasattr(self, 'mongo_client') and self.mongo_client:
            self.mongo_client.close()
        super().destroy_node()

def main(args=None):
    # 라이브러리 자동 설치 로직 (이전과 동일)
    try: pkg_resources.get_distribution("pymongo")
    except: subprocess.run(["pip", "install", "pymongo"], check=True)
    try: pkg_resources.get_distribution("dnspython")
    except: subprocess.run(["pip", "install", "dnspython"], check=True)

    rclpy.init(args=args)
    node = None
    try:
        node = YoloAlertNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # 종료는 finally 블록에서 우아하게 처리
    except Exception as e:
        if node: node.get_logger().error(f"🚨 노드 실행 중 오류 발생: {e}")
        else: print(f"🚨 노드 초기화 중 심각한 오류 발생: {e}")
    finally:
        if node:
            node.destroy_node() # 수정된 destroy_node 함수 호출
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

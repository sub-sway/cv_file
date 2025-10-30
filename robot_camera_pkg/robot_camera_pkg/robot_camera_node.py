#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image # ROS2 이미지 메시지 타입
from cv_bridge import CvBridge # ROS 이미지와 OpenCV 이미지 변환을 위한 브릿지


class ImagePublisherNode(Node):
    """
    USB 웹캠으로부터 영상을 읽어 ROS2 토픽으로 발행하는 노드입니다.
    """
    def __init__(self):
        super().__init__('image_publisher_node')
        self.get_logger().info("📷 Image Publisher Node Started")

        # --- 설정 값 ---
        self.camera_index = 0  # 사용할 카메라 인덱스 (0: 기본 웹캠)
        self.publish_rate = 30 # 1초에 30번 발행 (30 FPS)

        # --- ROS2 퍼블리셔 설정 ---
        # '/image_raw' 토픽에 Image 메시지 타입으로 발행
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # --- 카메라 및 CV 브릿지 설정 ---
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Could not open camera at index {self.camera_index}.")
            raise SystemExit

        self.bridge = CvBridge()
        self.get_logger().info(f"✅ Camera {self.camera_index} opened. Publishing images at {self.publish_rate} FPS.")


    def timer_callback(self):
        """
        타이머에 의해 주기적으로 호출되어 카메라 프레임을 읽고 토픽으로 발행합니다.
        """
        ret, frame = self.cap.read()
        if ret:
            # OpenCV 이미지(frame)를 ROS2 Image 메시지로 변환하여 발행
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(image_message)
        else:
            self.get_logger().warn("⚠️ Could not read frame from camera.")

    def destroy_node(self):
        """ 노드 종료 시 카메라 리소스를 해제합니다. """
        if self.cap.isOpened():
            self.cap.release()
        self.get_logger().info("🛑 Node Shutting Down")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 User requested shutdown (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')
        self.get_logger().info("🎥 영상 스트리밍 노드 시작")

        # USB 카메라 열기
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ 카메라를 열 수 없습니다. 장치 번호를 확인하세요.")
            return

        self.timer = self.create_timer(0.03, self.timer_callback)  # 약 30fps

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ 프레임을 읽을 수 없습니다.")
            return

        # 화면에 프레임 표시
        cv2.imshow("USB Camera Stream", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("🛑 종료 명령 수신 ('q' 키)")
            rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

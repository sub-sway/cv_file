#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject

class RtspStreamNode(Node):
    def __init__(self):
        super().__init__('rtsp_stream_node')
        self.get_logger().info("🚀 RTSP 스트림 노드 시작")

        Gst.init(None)
        self.server = GstRtspServer.RTSPServer()
        self.factory = self.create_rtsp_factory()
        self.mounts = self.server.get_mount_points()
        self.mounts.add_factory("/stream", self.factory)
        self.server.attach(None)
        self.get_logger().info("📡 RTSP 서버 실행 중: rtsp://0.0.0.0:8554/stream")

        self.loop = GObject.MainLoop()
        self.loop.run()

    def create_rtsp_factory(self):
        factory = GstRtspServer.RTSPMediaFactory()
        # Jetson Orin USB 카메라 파이프라인
        pipeline = (
            "v4l2src device=/dev/video0 ! video/x-raw, width=1280, height=720, framerate=30/1 "
            "! videoconvert ! nvv4l2h264enc bitrate=2000000 "
            "! h264parse ! rtph264pay name=pay0 pt=96"
        )
        factory.set_launch(pipeline)
        factory.set_shared(True)
        return factory


def main(args=None):
    rclpy.init(args=args)
    node = RtspStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import gi

gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject

class RtspStreamNode(Node):
    def __init__(self):
        super().__init__('rtsp_stream_node')
        self.get_logger().info("🚀 RTSP 스트림 노드 시작 (Jetson Orin 전용)")

        Gst.init(None)

        # RTSP 서버 생성
        self.server = GstRtspServer.RTSPServer()
        self.factory = self.create_rtsp_factory()
        self.mounts = self.server.get_mount_points()
        self.mounts.add_factory("/stream", self.factory)
        self.server.attach(None)

        self.get_logger().info("📡 RTSP 서버 실행 중: rtsp://0.0.0.0:8554/stream")

        # GObject 메인 루프 실행
        self.loop = GObject.MainLoop()
        self.loop.run()

    def create_rtsp_factory(self):
        factory = GstRtspServer.RTSPMediaFactory()

        # ✅ Jetson Orin용 하드웨어 인코딩 파이프라인
        pipeline = (
            "v4l2src device=/dev/video0 ! "
            "video/x-raw, width=1280, height=720, framerate=30/1 ! "
            "videoconvert ! "
            "nvv4l2h264enc bitrate=2000000 insert-sps-pps=true idrinterval=15 ! "
            "h264parse ! rtph264pay name=pay0 pt=96 config-interval=1"
        )

        factory.set_launch(pipeline)
        factory.set_shared(True)
        return factory


def main(args=None):
    rclpy.init(args=args)
    node = RtspStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 RTSP 노드 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

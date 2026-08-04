#!/usr/bin/env python3

"""Stream one ROS JPEG-compressed image topic as hardware H.264 WebRTC."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from yolo_webrtc_transport import YoloWebRtcTransport


def _jpeg_dimensions(data: bytes) -> tuple[int, int]:
    """Read JPEG dimensions without decoding the image."""
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        return 0, 0
    sof_markers = {
        0xC0, 0xC1, 0xC2, 0xC3,
        0xC5, 0xC6, 0xC7,
        0xC9, 0xCA, 0xCB,
        0xCD, 0xCE, 0xCF,
    }
    offset = 2
    while offset + 3 < len(data):
        if data[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in {0x01, 0xD8, 0xD9}:
            continue
        if offset + 2 > len(data):
            break
        segment_length = int.from_bytes(data[offset:offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(data):
            break
        if marker in sof_markers and segment_length >= 7:
            height = int.from_bytes(data[offset + 3:offset + 5], "big")
            width = int.from_bytes(data[offset + 5:offset + 7], "big")
            return width, height
        offset += segment_length
    return 0, 0


class RosImageWebRtcStreamer(Node):
    def __init__(self) -> None:
        super().__init__("ros_image_webrtc_streamer")
        self.declare_parameter("input_topic", "/image_raw/compressed")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 8091)
        self.declare_parameter("bitrate", 2_000_000)
        self.declare_parameter("iframe_interval", 10)
        self.declare_parameter("max_fps", 30.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self._last_bad_format_log_at = 0.0
        self._last_bad_jpeg_log_at = 0.0
        self.transport = YoloWebRtcTransport(
            host=str(self.get_parameter("bind_address").value),
            port=int(self.get_parameter("port").value),
            bitrate=int(self.get_parameter("bitrate").value),
            iframe_interval=int(self.get_parameter("iframe_interval").value),
            max_fps=float(self.get_parameter("max_fps").value),
            log_info=self.get_logger().info,
            log_warning=self.get_logger().warning,
            source_status=self._source_status,
        )
        if not self.transport.start():
            error = self.transport.startup_error or "unknown startup error"
            self.transport.stop()
            raise RuntimeError(f"failed to start H.264 WebRTC transport: {error}")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self._on_image,
            sensor_qos,
        )
        self.get_logger().info(
            f"Streaming {self.input_topic} as H.264 WebRTC on port "
            f"{int(self.get_parameter('port').value)}"
        )

    def _source_status(self) -> dict:
        return {
            "selected": self.input_topic,
            "mode": "video_only",
            "sources": [
                {
                    "topic": self.input_topic,
                    "available": True,
                    "yolo_synchronized": False,
                }
            ],
        }

    def _on_image(self, msg: CompressedImage) -> None:
        image_format = str(msg.format).lower()
        if "jpeg" not in image_format and "jpg" not in image_format:
            now = time.monotonic()
            if now - self._last_bad_format_log_at >= 2.0:
                self._last_bad_format_log_at = now
                self.get_logger().warning(
                    f"Expected JPEG-compressed image on {self.input_topic}, got "
                    f"{msg.format!r}"
                )
            return

        jpeg_data = bytes(msg.data)
        width, height = _jpeg_dimensions(jpeg_data)
        if width <= 0 or height <= 0:
            now = time.monotonic()
            if now - self._last_bad_jpeg_log_at >= 2.0:
                self._last_bad_jpeg_log_at = now
                self.get_logger().warning(
                    f"Could not read JPEG dimensions from {self.input_topic}"
                )
            return

        stamp_sec = int(msg.header.stamp.sec)
        stamp_nanosec = int(msg.header.stamp.nanosec)
        if stamp_sec == 0 and stamp_nanosec == 0:
            stamp_ns = self.get_clock().now().nanoseconds
            stamp_sec, stamp_nanosec = divmod(stamp_ns, 1_000_000_000)
        self.transport.push_frame(
            jpeg_data=jpeg_data,
            stamp_sec=stamp_sec,
            stamp_nanosec=stamp_nanosec,
            width=width,
            height=height,
            detections=[],
        )

    def destroy_node(self) -> bool:
        self.transport.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RosImageWebRtcStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

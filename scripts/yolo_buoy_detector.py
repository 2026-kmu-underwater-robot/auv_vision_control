#!/usr/bin/env python3
import importlib
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from yolo_webrtc_transport import YoloWebRtcTransport


BBOX_FORMAT = (
    "data = [stamp_sec, detected, class_id, confidence, center_x, center_y, "
    "width, height, image_width, image_height]"
)
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
YOLO_H264_SOURCE_TOPIC = "/vision/yolo/annotated/compressed"
PERFORMANCE_LOG_INTERVAL_SEC = 5.0
PERFORMANCE_CUDA_SAMPLE_EVERY_N = 10
PERFORMANCE_WARMUP_FRAMES = 5
DECODE_PREFETCH_DELAY_SEC = 0.020


@dataclass(frozen=True)
class _CompressedFrame:
    jpeg_data: bytes
    image_format: str
    stamp_sec: int
    stamp_nanosec: int
    received_at: float


@dataclass(frozen=True)
class _DecodedFrame:
    image: np.ndarray
    compressed: _CompressedFrame
    decode_queue_wait_ms: float
    jpeg_decode_ms: float
    decoded_at: float


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


class YoloBuoyDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_buoy_detector")

        self.declare_parameter("image_topic", "/imx219/camera0/image_raw/compressed")
        self.declare_parameter("bbox_topic", "/vision/buoy_bbox")
        self.declare_parameter("model_path", "/home/auv/models/buoy.pt")
        self.declare_parameter("target_class_id", 0)
        self.declare_parameter("target_class_name", "")
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("device", "auto")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("preview_window_name", "YOLO Buoy Detection")
        self.declare_parameter("publish_per_class", False)
        self.declare_parameter("webrtc_enabled", True)
        self.declare_parameter("webrtc_bind_address", "0.0.0.0")
        self.declare_parameter("webrtc_port", 8090)
        self.declare_parameter("webrtc_bitrate", 2_000_000)
        self.declare_parameter("webrtc_iframe_interval", 10)
        self.declare_parameter("webrtc_max_fps", 30.0)
        # 다중 부표 선택: 면적 큰 것 → 박스 확률(confidence) → 이미지 오른쪽
        self.declare_parameter("area_similar_ratio", 0.15)
        self.declare_parameter("confidence_similar_delta", 0.05)

        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.model_path = self.get_parameter("model_path").value
        if not self.model_path or not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"model_path not found: {self.model_path!r}. "
                "Pass model_path:=/absolute/path/to/model.pt"
            )
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip().lower()
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.device = self._resolve_device(str(self.get_parameter("device").value))
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.preview_window_name = str(self.get_parameter("preview_window_name").value)
        self.publish_per_class = bool(self.get_parameter("publish_per_class").value)
        self.webrtc_enabled = bool(self.get_parameter("webrtc_enabled").value)
        self.webrtc_bind_address = str(
            self.get_parameter("webrtc_bind_address").value
        ).strip()
        self.webrtc_port = int(self.get_parameter("webrtc_port").value)
        self.webrtc_bitrate = int(self.get_parameter("webrtc_bitrate").value)
        self.webrtc_iframe_interval = int(
            self.get_parameter("webrtc_iframe_interval").value
        )
        self.webrtc_max_fps = float(self.get_parameter("webrtc_max_fps").value)
        self.area_similar_ratio = float(self.get_parameter("area_similar_ratio").value)
        self.confidence_similar_delta = float(
            self.get_parameter("confidence_similar_delta").value
        )
        self._preview_prev_time: Optional[float] = None
        self._preview_fps = 0.0
        self._performance_samples: deque[dict[str, float]] = deque(maxlen=300)
        self._performance_frame_count = 0
        self._performance_last_log_at = time.monotonic()
        self._performance_torch = None
        self._performance_cuda_enabled = False
        self._pipeline_stop_event = threading.Event()
        self._decode_condition = threading.Condition()
        self._decode_pending: Optional[_CompressedFrame] = None
        self._inference_condition = threading.Condition()
        self._inference_pending: Optional[_DecodedFrame] = None
        self._inference_prefetch_available = threading.Event()
        self._inference_prefetch_available.set()
        self._decode_not_before = 0.0
        self._pipeline_stats_lock = threading.Lock()
        self._pipeline_received = 0
        self._pipeline_decoded = 0
        self._pipeline_processed = 0
        self._pipeline_input_dropped = 0
        self._pipeline_decoded_dropped = 0
        self._decode_thread: Optional[threading.Thread] = None
        self._inference_thread: Optional[threading.Thread] = None
        self._webrtc_source_lock = threading.Lock()
        self._webrtc_source_topic = YOLO_H264_SOURCE_TOPIC
        self._webrtc_source_pending = ""
        self._webrtc_source_error = ""
        self._webrtc_camera_subscription = None
        self._webrtc_source_signature = ""
        self._webrtc_sources = [
            {
                "topic": YOLO_H264_SOURCE_TOPIC,
                "type": COMPRESSED_IMAGE_TYPE,
                "available": True,
                "virtual": True,
                "yolo_synchronized": True,
            },
            {
                "topic": str(self.image_topic),
                "type": COMPRESSED_IMAGE_TYPE,
                "available": True,
                "virtual": False,
                "yolo_synchronized": False,
            }
        ]

        self.model = self._load_model(self.model_path)
        self.class_names = getattr(self.model, "names", {}) or {}
        self._warn_if_target_class_mismatch()
        self._configure_performance_profiling()

        self.image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._inference_callback_group = MutuallyExclusiveCallbackGroup()
        self._stream_callback_group = MutuallyExclusiveCallbackGroup()
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.on_image,
            self.image_qos,
            callback_group=self._inference_callback_group,
        )
        self.bbox_pub = self.create_publisher(Float32MultiArray, self.bbox_topic, 10)
        self.webrtc_transport: Optional[YoloWebRtcTransport] = None
        if self.webrtc_enabled:
            self._start_webrtc_transport()
            self._webrtc_source_timer = self.create_timer(
                1.0,
                self._refresh_webrtc_sources,
                callback_group=self._stream_callback_group,
            )

        self.get_logger().info(f"Subscribing: {self.image_topic}")
        self.get_logger().info(f"Publishing: {self.bbox_topic} ({BBOX_FORMAT})")
        self.get_logger().info(f"Model classes: {self._format_class_names()}")
        self.get_logger().info(
            f"YOLO PT model={self.model_path}, device={self.device}, imgsz={self.imgsz}, "
            f"target_class_id={self.target_class_id}, target_class_name='{self.target_class_name}', "
            f"show_preview={self.show_preview}, publish_per_class={self.publish_per_class}"
        )
        if self.show_preview:
            self.get_logger().info(
                f"Preview window '{self.preview_window_name}' enabled (press q in window to quit)"
            )
        self._start_pipeline_workers()

    def _start_pipeline_workers(self) -> None:
        self._decode_thread = threading.Thread(
            target=self._decode_worker,
            name="yolo-jpeg-decode",
            daemon=True,
        )
        self._inference_thread = threading.Thread(
            target=self._inference_worker,
            name="yolo-gpu-inference",
            daemon=True,
        )
        self._decode_thread.start()
        self._inference_thread.start()
        self.get_logger().info(
            "YOLO pipelined scheduler enabled: ROS receive -> JPEG decode -> GPU "
            "inference, latest-frame queues depth=1, "
            f"decode_prefetch_delay={DECODE_PREFETCH_DELAY_SEC * 1000.0:.0f}ms"
        )

    def _start_webrtc_transport(self) -> None:
        try:
            transport = YoloWebRtcTransport(
                host=self.webrtc_bind_address,
                port=self.webrtc_port,
                bitrate=self.webrtc_bitrate,
                iframe_interval=self.webrtc_iframe_interval,
                max_fps=self.webrtc_max_fps,
                log_info=self.get_logger().info,
                log_warning=self.get_logger().warning,
                source_status=self._webrtc_source_status,
                select_source=self._request_webrtc_source,
            )
            if transport.start():
                self.webrtc_transport = transport
                return
            message = transport.startup_error or "unknown startup error"
            transport.stop()
            self.get_logger().warning(
                f"H.264 WebRTC unavailable; detector continues without web video: {message}"
            )
        except Exception as exc:
            self.get_logger().warning(
                "H.264 WebRTC unavailable; detector continues without web video: "
                f"{exc}"
            )

    def _webrtc_source_status(self) -> dict:
        transport = getattr(self, "webrtc_transport", None)
        with self._webrtc_source_lock:
            selected = self._webrtc_source_topic
            return {
                "selected": selected,
                "pending": self._webrtc_source_pending,
                "mode": (
                    "yolo_synchronized"
                    if selected == YOLO_H264_SOURCE_TOPIC
                    else "video_only"
                ),
                "sources": [dict(item) for item in self._webrtc_sources],
                "error": self._webrtc_source_error,
                "encoded_fps": (
                    round(float(transport.encoded_fps), 3)
                    if transport is not None
                    else 0.0
                ),
                "viewers": transport.peer_count if transport is not None else 0,
            }

    def _request_webrtc_source(self, topic: str) -> dict:
        requested = str(topic).strip()
        if not requested:
            raise ValueError("H.264 source topic is required")
        with self._webrtc_source_lock:
            available_topics = {
                str(item["topic"])
                for item in self._webrtc_sources
                if item.get("available", False)
            }
            if requested not in available_topics:
                raise ValueError(
                    f"H.264 source is not an available JPEG CompressedImage topic: {requested}"
                )
            self._webrtc_source_pending = requested
            self._webrtc_source_error = ""
        return {"accepted": True, "pending": requested}

    def _refresh_webrtc_sources(self) -> None:
        topic_names_and_types = dict(self.get_topic_names_and_types())
        camera_available = (
            COMPRESSED_IMAGE_TYPE
            in topic_names_and_types.get(self.image_topic, [])
        )
        sources = [
            {
                "topic": YOLO_H264_SOURCE_TOPIC,
                "type": COMPRESSED_IMAGE_TYPE,
                "available": camera_available,
                "virtual": True,
                "yolo_synchronized": True,
            },
            {
                "topic": str(self.image_topic),
                "type": COMPRESSED_IMAGE_TYPE,
                "available": camera_available,
                "virtual": False,
                "yolo_synchronized": False,
            },
        ]

        with self._webrtc_source_lock:
            current = self._webrtc_source_topic
            pending = self._webrtc_source_pending
            available_topics = {
                str(item["topic"])
                for item in sources
                if item["available"]
            }
            requested = pending or current
            error = ""
            if requested not in available_topics:
                error = f"H.264 source unavailable: {requested}"
                requested = YOLO_H264_SOURCE_TOPIC
            self._webrtc_sources = sources
            self._webrtc_source_pending = ""

        if requested != current:
            try:
                self._switch_webrtc_source(requested)
                self.get_logger().info(f"H.264 source switched to: {requested}")
            except Exception as exc:
                error = f"Failed to switch H.264 source: {exc}"
                self.get_logger().warning(error)

        with self._webrtc_source_lock:
            self._webrtc_source_error = error
            signature = repr(
                (
                    self._webrtc_source_topic,
                    self._webrtc_source_error,
                    self._webrtc_sources,
                )
            )
            changed = signature != self._webrtc_source_signature
            self._webrtc_source_signature = signature
        if changed and self.webrtc_transport is not None:
            self.webrtc_transport.broadcast_source_status()

    def _switch_webrtc_source(self, topic: str) -> None:
        new_camera_subscription = None
        if topic == self.image_topic:
            new_camera_subscription = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self._on_webrtc_camera_image,
                self.image_qos,
                callback_group=self._stream_callback_group,
            )
        old_camera_subscription = self._webrtc_camera_subscription
        self._webrtc_camera_subscription = new_camera_subscription
        with self._webrtc_source_lock:
            self._webrtc_source_topic = topic
        if old_camera_subscription is not None:
            self.destroy_subscription(old_camera_subscription)
        if self.webrtc_transport is not None:
            self.webrtc_transport.reset_source_timing()

    def _on_webrtc_camera_image(self, msg: CompressedImage) -> None:
        transport = self.webrtc_transport
        if transport is None:
            return
        with self._webrtc_source_lock:
            if self._webrtc_source_topic != self.image_topic:
                return
        image_format = str(msg.format).lower()
        if "jpeg" not in image_format and "jpg" not in image_format:
            self.get_logger().warning(
                f"Camera H.264 source is not JPEG-compressed: {msg.format!r}",
                throttle_duration_sec=2.0,
            )
            return
        jpeg_data = bytes(msg.data)
        width, height = _jpeg_dimensions(jpeg_data)
        if width <= 0 or height <= 0:
            self.get_logger().warning(
                "Could not read camera JPEG dimensions",
                throttle_duration_sec=2.0,
            )
            return
        transport.push_frame(
            jpeg_data=jpeg_data,
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
            width=width,
            height=height,
            detections=[],
        )

    def _load_model(self, model_path: str):
        ultralytics = importlib.import_module("ultralytics")
        return ultralytics.YOLO(model_path)

    def _configure_performance_profiling(self) -> None:
        if not str(self.device).startswith("cuda"):
            return
        try:
            torch = importlib.import_module("torch")
            if torch.cuda.is_available():
                self._performance_torch = torch
                self._performance_cuda_enabled = True
                self.get_logger().info(
                    "YOLO performance profiling enabled: CUDA event sample every "
                    f"{PERFORMANCE_CUDA_SAMPLE_EVERY_N} frames, rolling log every "
                    f"{PERFORMANCE_LOG_INTERVAL_SEC:.0f} s"
                )
        except Exception as exc:
            self.get_logger().warning(
                f"CUDA event profiling unavailable; wall timings remain enabled: {exc}"
            )

    def _resolve_device(self, device: str) -> str:
        requested = device.strip()
        if requested and requested.lower() != "auto":
            return requested

        try:
            torch = importlib.import_module("torch")
            if torch.cuda.is_available():
                return "cuda:0"
        except ImportError:
            pass
        return "cpu"

    def _format_class_names(self) -> str:
        if not self.class_names:
            return "[]"
        return str({int(key): str(value) for key, value in self.class_names.items()})

    def _warn_if_target_class_mismatch(self) -> None:
        if self.target_class_id >= 0:
            if self.target_class_id not in self.class_names:
                self.get_logger().warning(
                    f"target_class_id={self.target_class_id} is not in model classes: "
                    f"{self._format_class_names()}"
                )
            return

        if not self.target_class_name:
            return

        model_class_names = {str(value).strip().lower() for value in self.class_names.values()}
        if self.target_class_name not in model_class_names:
            self.get_logger().warning(
                f"target_class_name='{self.target_class_name}' does not match model classes "
                f"{sorted(model_class_names)}. All detections will be filtered out unless you change "
                f"target_class_name or set target_class_name:="
            )

    def on_image(self, msg: CompressedImage) -> None:
        frame = _CompressedFrame(
            jpeg_data=bytes(msg.data),
            image_format=str(msg.format),
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
            received_at=time.perf_counter(),
        )
        with self._pipeline_stats_lock:
            self._pipeline_received += 1
        with self._decode_condition:
            if self._decode_pending is not None:
                with self._pipeline_stats_lock:
                    self._pipeline_input_dropped += 1
            self._decode_pending = frame
            self._decode_condition.notify()

    def _decode_worker(self) -> None:
        while not self._pipeline_stop_event.is_set():
            if not self._inference_prefetch_available.wait(timeout=0.1):
                continue
            if self._pipeline_stop_event.is_set():
                return
            prefetch_delay = self._decode_not_before - time.perf_counter()
            if (
                prefetch_delay > 0.0
                and self._pipeline_stop_event.wait(timeout=prefetch_delay)
            ):
                return
            with self._decode_condition:
                self._decode_condition.wait_for(
                    lambda: (
                        self._decode_pending is not None
                        or self._pipeline_stop_event.is_set()
                    )
                )
                if self._pipeline_stop_event.is_set():
                    return
                if not self._inference_prefetch_available.is_set():
                    continue
                self._inference_prefetch_available.clear()
                frame = self._decode_pending
                self._decode_pending = None
            if frame is None:
                continue

            decode_started_at = time.perf_counter()
            image = self._decode_jpeg_data(frame.jpeg_data)
            decoded_at = time.perf_counter()
            if image is None:
                self.get_logger().warning(
                    "Failed to decode compressed image",
                    throttle_duration_sec=2.0,
                )
                self._inference_prefetch_available.set()
                continue

            decoded = _DecodedFrame(
                image=image,
                compressed=frame,
                decode_queue_wait_ms=(decode_started_at - frame.received_at) * 1000.0,
                jpeg_decode_ms=(decoded_at - decode_started_at) * 1000.0,
                decoded_at=decoded_at,
            )
            with self._pipeline_stats_lock:
                self._pipeline_decoded += 1
            with self._inference_condition:
                if self._inference_pending is not None:
                    with self._pipeline_stats_lock:
                        self._pipeline_decoded_dropped += 1
                self._inference_pending = decoded
                self._inference_condition.notify()

    def _inference_worker(self) -> None:
        while not self._pipeline_stop_event.is_set():
            with self._inference_condition:
                self._inference_condition.wait_for(
                    lambda: (
                        self._inference_pending is not None
                        or self._pipeline_stop_event.is_set()
                    )
                )
                if self._pipeline_stop_event.is_set():
                    return
                frame = self._inference_pending
                self._inference_pending = None
                self._decode_not_before = (
                    time.perf_counter() + DECODE_PREFETCH_DELAY_SEC
                )
                self._inference_prefetch_available.set()
            if frame is None:
                continue

            try:
                self._process_decoded_frame(frame)
                with self._pipeline_stats_lock:
                    self._pipeline_processed += 1
            except Exception as exc:
                self.get_logger().error(
                    f"YOLO inference pipeline frame failed: {exc}",
                    throttle_duration_sec=2.0,
                )

    def _process_decoded_frame(self, frame: _DecodedFrame) -> None:
        inference_started_at = time.perf_counter()
        msg = frame.compressed
        image = frame.image
        image_height, image_width = image.shape[:2]
        detection, all_detections, performance = self._detect_targets(image)
        if self._pipeline_stop_event.is_set() or not rclpy.ok():
            return
        tail_started_at = time.perf_counter()
        stamp_sec = float(msg.stamp_sec) + float(msg.stamp_nanosec) * 1e-9

        published_detections = [detection] if detection is not None else []
        if self.publish_per_class:
            published_detections = self._best_detection_per_class(all_detections)

        if published_detections:
            for published_detection in published_detections:
                self._publish_detection(
                    stamp_sec, published_detection, image_width, image_height
                )
        else:
            self._publish_detection(stamp_sec, None, image_width, image_height)

        with self._webrtc_source_lock:
            stream_inference_frame = (
                self._webrtc_source_topic == YOLO_H264_SOURCE_TOPIC
            )
        if self.webrtc_transport is not None and stream_inference_frame:
            self.webrtc_transport.push_frame(
                jpeg_data=msg.jpeg_data,
                stamp_sec=msg.stamp_sec,
                stamp_nanosec=msg.stamp_nanosec,
                width=image_width,
                height=image_height,
                detections=self._display_detections(detection, all_detections),
            )

        if self.show_preview:
            self._update_preview_fps()
            annotated_image = self._render_annotated_image(
                image, detection, all_detections
            )
            self._show_preview(annotated_image)

        completed_at = time.perf_counter()
        performance["jpeg_decode_ms"] = frame.jpeg_decode_ms
        performance["decode_queue_wait_ms"] = frame.decode_queue_wait_ms
        performance["inference_queue_wait_ms"] = (
            inference_started_at - frame.decoded_at
        ) * 1000.0
        performance["callback_tail_ms"] = (
            completed_at - tail_started_at
        ) * 1000.0
        performance["callback_total_ms"] = (
            completed_at - msg.received_at
        ) * 1000.0
        performance["inference_cycle_ms"] = (
            completed_at - inference_started_at
        ) * 1000.0
        self._record_performance_sample(performance)

    def _display_detections(
        self,
        selected: Optional[Tuple[int, float, float, float, float, float]],
        all_detections: list[
            Tuple[int, float, float, float, float, float, int, int, int, int]
        ],
    ) -> list[dict]:
        selected_key = None
        if selected is not None:
            selected_key = (
                selected[0],
                round(selected[1], 4),
                round(selected[2], 1),
                round(selected[3], 1),
            )
        detections = []
        for class_id, confidence, center_x, center_y, width, height, *_ in all_detections:
            key = (
                class_id,
                round(confidence, 4),
                round(center_x, 1),
                round(center_y, 1),
            )
            detections.append(
                {
                    "class_id": int(class_id),
                    "class_name": str(self.class_names.get(class_id, class_id)),
                    "confidence": round(float(confidence), 6),
                    "center_x": round(float(center_x), 3),
                    "center_y": round(float(center_y), 3),
                    "width": round(float(width), 3),
                    "height": round(float(height), 3),
                    "selected": key == selected_key,
                }
            )
        return detections

    def _decode_compressed_image(self, msg: CompressedImage) -> Optional[np.ndarray]:
        return self._decode_jpeg_data(bytes(msg.data))

    def _decode_jpeg_data(self, jpeg_data: bytes) -> Optional[np.ndarray]:
        data = np.frombuffer(jpeg_data, dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR)

    def _is_better_detection(
        self,
        candidate: Tuple[int, float, float, float, float, float],
        current: Tuple[int, float, float, float, float, float],
    ) -> bool:
        """면적 큰 것 → 박스 확률(confidence) → 이미지 오른쪽(center_x) 순으로 비교."""
        _, cand_conf, cand_cx, _, cand_w, cand_h = candidate
        _, cur_conf, cur_cx, _, cur_w, cur_h = current
        cand_area = max(0.0, cand_w * cand_h)
        cur_area = max(0.0, cur_w * cur_h)
        larger = max(cand_area, cur_area, 1.0)
        if abs(cand_area - cur_area) > self.area_similar_ratio * larger:
            return cand_area > cur_area
        if abs(cand_conf - cur_conf) > self.confidence_similar_delta:
            return cand_conf > cur_conf
        return cand_cx > cur_cx

    def _best_detection_per_class(
        self,
        all_detections: list[Tuple[int, float, float, float, float, float, int, int, int, int]],
    ) -> list[Tuple[int, float, float, float, float, float]]:
        best_by_class: dict[int, Tuple[int, float, float, float, float, float]] = {}
        for class_id, confidence, center_x, center_y, width, height, *_ in all_detections:
            if not self._class_matches(class_id):
                continue
            candidate = (
                class_id,
                confidence,
                center_x,
                center_y,
                width,
                height,
            )
            previous = best_by_class.get(class_id)
            if previous is None or self._is_better_detection(candidate, previous):
                best_by_class[class_id] = candidate
        return [best_by_class[class_id] for class_id in sorted(best_by_class)]

    def _publish_detection(
        self,
        stamp_sec: float,
        detection: Optional[Tuple[int, float, float, float, float, float]],
        image_width: int,
        image_height: int,
    ) -> None:
        out = Float32MultiArray()
        if detection is None:
            out.data = [
                stamp_sec, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                float(image_width), float(image_height),
            ]
        else:
            class_id, confidence, center_x, center_y, width, height = detection
            out.data = [
                stamp_sec, 1.0, float(class_id), float(confidence),
                float(center_x), float(center_y), float(width), float(height),
                float(image_width), float(image_height),
            ]
        self.bbox_pub.publish(out)

    def _detect_targets(
        self, image: np.ndarray
    ) -> Tuple[
        Optional[Tuple[int, float, float, float, float, float]],
        list[Tuple[int, float, float, float, float, float, int, int, int, int]],
        dict[str, float],
    ]:
        self._performance_frame_count += 1
        profile_cuda = (
            self._performance_cuda_enabled
            and self._performance_frame_count % PERFORMANCE_CUDA_SAMPLE_EVERY_N == 0
        )
        cuda_start = None
        cuda_end = None
        if profile_cuda:
            try:
                cuda_start = self._performance_torch.cuda.Event(enable_timing=True)
                cuda_end = self._performance_torch.cuda.Event(enable_timing=True)
                cuda_start.record()
            except Exception as exc:
                self._performance_cuda_enabled = False
                self.get_logger().warning(f"CUDA event profiling disabled: {exc}")
                cuda_start = None
                cuda_end = None

        predict_started_at = time.perf_counter()
        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )
        predict_wall_ms = (time.perf_counter() - predict_started_at) * 1000.0
        cuda_predict_span_ms = -1.0
        if cuda_start is not None and cuda_end is not None:
            try:
                cuda_end.record()
                cuda_end.synchronize()
                cuda_predict_span_ms = float(cuda_start.elapsed_time(cuda_end))
            except Exception as exc:
                self._performance_cuda_enabled = False
                self.get_logger().warning(f"CUDA event profiling disabled: {exc}")

        speed = getattr(results[0], "speed", {}) if results else {}
        performance = {
            "predict_wall_ms": predict_wall_ms,
            "cuda_predict_span_ms": cuda_predict_span_ms,
            "preprocess_h2d_ms": float(speed.get("preprocess", 0.0) or 0.0),
            "inference_ms": float(speed.get("inference", 0.0) or 0.0),
            "yolo_postprocess_ms": float(speed.get("postprocess", 0.0) or 0.0),
            "d2h_ms": 0.0,
            "bbox_cpu_ms": 0.0,
        }
        if not results:
            return None, [], performance

        boxes = getattr(results[0], "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None, [], performance

        d2h_started_at = time.perf_counter()
        xyxy = boxes.xyxy.detach().cpu().numpy()
        confidences = boxes.conf.detach().cpu().numpy()
        classes = boxes.cls.detach().cpu().numpy().astype(int)
        performance["d2h_ms"] = (time.perf_counter() - d2h_started_at) * 1000.0

        bbox_started_at = time.perf_counter()
        all_detections: list[Tuple[int, float, float, float, float, float, int, int, int, int]] = []
        best = None
        filtered_count = 0
        for rect, confidence, class_id in zip(xyxy, confidences, classes):
            x1, y1, x2, y2 = rect
            width = max(0.0, float(x2 - x1))
            height = max(0.0, float(y2 - y1))
            center_x = float(x1 + width / 2.0)
            center_y = float(y1 + height / 2.0)
            ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
            det = (
                int(class_id),
                float(confidence),
                center_x,
                center_y,
                width,
                height,
                ix1,
                iy1,
                ix2,
                iy2,
            )
            all_detections.append(det)

            if not self._class_matches(int(class_id)):
                filtered_count += 1
                continue
            candidate = (
                int(class_id),
                float(confidence),
                center_x,
                center_y,
                width,
                height,
            )
            if best is None or self._is_better_detection(candidate, best):
                best = candidate

        if all_detections and best is None:
            self.get_logger().warning(
                f"YOLO found {len(all_detections)} object(s) but none matched "
                f"target_class_id={self.target_class_id}, target_class_name='{self.target_class_name}'. "
                f"Filtered {filtered_count} detection(s).",
                throttle_duration_sec=3.0,
            )

        performance["bbox_cpu_ms"] = (
            time.perf_counter() - bbox_started_at
        ) * 1000.0
        return best, all_detections, performance

    def _record_performance_sample(self, performance: dict[str, float]) -> None:
        if self._performance_frame_count <= PERFORMANCE_WARMUP_FRAMES:
            return
        sample = {
            key: float(value)
            for key, value in performance.items()
            if isinstance(value, (int, float))
        }
        sample["recorded_at"] = time.monotonic()
        self._performance_samples.append(sample)

        now = time.monotonic()
        if now - self._performance_last_log_at < PERFORMANCE_LOG_INTERVAL_SEC:
            return
        self._performance_last_log_at = now
        samples = list(self._performance_samples)
        if len(samples) < 2:
            return

        elapsed = samples[-1]["recorded_at"] - samples[0]["recorded_at"]
        measured_fps = (len(samples) - 1) / elapsed if elapsed > 0.0 else 0.0

        def stats(key: str, *, positive_only: bool = False) -> tuple[float, float, int]:
            values = [
                item[key]
                for item in samples
                if key in item and (not positive_only or item[key] >= 0.0)
            ]
            if not values:
                return 0.0, 0.0, 0
            return (
                float(np.mean(values)),
                float(np.percentile(values, 95)),
                len(values),
            )

        callback_avg, callback_p95, _ = stats("callback_total_ms")
        decode_avg, decode_p95, _ = stats("jpeg_decode_ms")
        decode_wait_avg, decode_wait_p95, _ = stats("decode_queue_wait_ms")
        inference_wait_avg, inference_wait_p95, _ = stats("inference_queue_wait_ms")
        cycle_avg, cycle_p95, _ = stats("inference_cycle_ms")
        predict_avg, predict_p95, _ = stats("predict_wall_ms")
        preprocess_avg, preprocess_p95, _ = stats("preprocess_h2d_ms")
        inference_avg, inference_p95, _ = stats("inference_ms")
        yolo_post_avg, yolo_post_p95, _ = stats("yolo_postprocess_ms")
        d2h_avg, d2h_p95, _ = stats("d2h_ms")
        bbox_avg, bbox_p95, _ = stats("bbox_cpu_ms")
        tail_avg, tail_p95, _ = stats("callback_tail_ms")
        cuda_avg, cuda_p95, cuda_count = stats(
            "cuda_predict_span_ms",
            positive_only=True,
        )
        with self._pipeline_stats_lock:
            received = self._pipeline_received
            decoded = self._pipeline_decoded
            processed = self._pipeline_processed
            input_dropped = self._pipeline_input_dropped
            decoded_dropped = self._pipeline_decoded_dropped

        self.get_logger().info(
            "YOLO PERF "
            f"n={len(samples)} rate={measured_fps:.2f}fps "
            f"latency={callback_avg:.2f}/{callback_p95:.2f}ms(avg/p95) "
            f"cycle={cycle_avg:.2f}/{cycle_p95:.2f}ms "
            f"jpeg_decode={decode_avg:.2f}/{decode_p95:.2f}ms "
            f"predict_wall={predict_avg:.2f}/{predict_p95:.2f}ms "
            f"tail={tail_avg:.2f}/{tail_p95:.2f}ms"
        )
        self.get_logger().info(
            "YOLO PIPELINE "
            f"decode_wait={decode_wait_avg:.2f}/{decode_wait_p95:.2f}ms "
            f"inference_wait={inference_wait_avg:.2f}/{inference_wait_p95:.2f}ms "
            f"received={received} decoded={decoded} processed={processed} "
            f"drop_before_decode={input_dropped} drop_before_inference={decoded_dropped}"
        )
        self.get_logger().info(
            "YOLO STAGES "
            f"preprocess_h2d={preprocess_avg:.2f}/{preprocess_p95:.2f}ms "
            f"inference={inference_avg:.2f}/{inference_p95:.2f}ms "
            f"yolo_post={yolo_post_avg:.2f}/{yolo_post_p95:.2f}ms "
            f"d2h={d2h_avg:.2f}/{d2h_p95:.2f}ms "
            f"bbox_cpu={bbox_avg:.2f}/{bbox_p95:.2f}ms "
            f"cuda_predict_span={cuda_avg:.2f}/{cuda_p95:.2f}ms(samples={cuda_count})"
        )

    def _class_matches(self, class_id: int) -> bool:
        if self.target_class_id >= 0:
            return class_id == self.target_class_id
        if not self.target_class_name:
            return True
        class_name = str(self.class_names.get(class_id, "")).strip().lower()
        return class_name == self.target_class_name

    def _update_preview_fps(self) -> None:
        now = time.monotonic()
        if self._preview_prev_time is not None:
            dt = now - self._preview_prev_time
            if dt > 0.0:
                self._preview_fps = 1.0 / dt
        self._preview_prev_time = now

    def _render_annotated_image(
        self,
        image: np.ndarray,
        detection: Optional[Tuple[int, float, float, float, float, float]],
        all_detections: list[Tuple[int, float, float, float, float, float, int, int, int, int]],
    ) -> np.ndarray:
        display = image.copy()
        height, width = display.shape[:2]
        center_x = width // 2
        center_y = height // 2

        selected_key = None
        if detection is not None:
            selected_key = (
                detection[0],
                round(detection[1], 4),
                round(detection[2], 1),
                round(detection[3], 1),
            )

        for class_id, confidence, det_cx, det_cy, _, _, x1, y1, x2, y2 in all_detections:
            det_key = (class_id, round(confidence, 4), round(det_cx, 1), round(det_cy, 1))
            is_selected = selected_key == det_key
            color = (0, 255, 0) if is_selected else (0, 165, 255)
            thickness = 2 if is_selected else 1
            cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
            class_name = str(self.class_names.get(class_id, class_id))
            label = f"{class_name} {confidence:.2f}"
            cv2.putText(
                display,
                label,
                (x1, max(18, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )

        cv2.drawMarker(
            display,
            (center_x, center_y),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=1,
        )
        cv2.line(display, (center_x, 0), (center_x, height), (0, 255, 255), 1)
        cv2.line(display, (0, center_y), (width, center_y), (0, 255, 255), 1)

        status = "NO DETECTION"
        status_color = (0, 0, 255)
        if detection is not None:
            class_id, confidence, det_cx, det_cy, _, _ = detection
            cv2.drawMarker(
                display,
                (int(det_cx), int(det_cy)),
                (0, 255, 0),
                markerType=cv2.MARKER_TILTED_CROSS,
                markerSize=16,
                thickness=2,
            )
            class_name = str(self.class_names.get(class_id, class_id))
            status = f"TRACKING {class_name} {confidence:.2f}"
            status_color = (0, 255, 0)
        elif all_detections:
            status = f"FILTERED {len(all_detections)} detection(s)"
            status_color = (0, 165, 255)

        cv2.putText(
            display,
            status,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            f"FPS {self._preview_fps:.1f}  device={self.device}",
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            f"{width}x{height}  topic={self.image_topic}",
            (12, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            display,
            f"raw={len(all_detections)}  filter='{self.target_class_name or 'ALL'}'",
            (12, 108),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )

        return display

    def _show_preview(self, display: np.ndarray) -> None:
        cv2.imshow(self.preview_window_name, display)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            raise KeyboardInterrupt("Preview window closed by user")

    def close_preview(self) -> None:
        if self.show_preview:
            cv2.destroyWindow(self.preview_window_name)

    def destroy_node(self) -> bool:
        self._stop_pipeline_workers()
        if self.webrtc_transport is not None:
            self.webrtc_transport.stop()
            self.webrtc_transport = None
        return super().destroy_node()

    def _stop_pipeline_workers(self) -> None:
        self._pipeline_stop_event.set()
        self._inference_prefetch_available.set()
        with self._decode_condition:
            self._decode_condition.notify_all()
        with self._inference_condition:
            self._inference_condition.notify_all()
        current_thread = threading.current_thread()
        for worker in (self._decode_thread, self._inference_thread):
            if worker is not None and worker is not current_thread and worker.is_alive():
                worker.join(timeout=3.0)
        self._decode_thread = None
        self._inference_thread = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloBuoyDetector()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_pipeline_workers()
        executor.shutdown()
        node.close_preview()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

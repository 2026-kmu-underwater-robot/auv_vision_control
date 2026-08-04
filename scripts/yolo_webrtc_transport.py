#!/usr/bin/env python3

"""Optional H.264/WebRTC transport for YOLO source frames and detections."""

from __future__ import annotations

import asyncio
import json
import os
import threading
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass(eq=False)
class _Peer:
    peer_id: str
    websocket: object
    outgoing: asyncio.Queue
    queue: object | None = None
    webrtc: object | None = None
    tee_pad: object | None = None
    webrtc_pad: object | None = None
    signal_handlers: list[int] | None = None
    offer_started: bool = False
    remote_description_set: bool = False
    pending_ice: list[tuple[int, str]] | None = None
    local_offer_sdp: str = ""
    remote_answer: object | None = None
    remote_answer_promise: object | None = None


class YoloWebRtcTransport:
    """Encode JPEG source frames once with NVENC and fan RTP out to WebRTC peers."""

    RTP_CLOCK_RATE = 90_000

    def __init__(
        self,
        host: str,
        port: int,
        bitrate: int,
        iframe_interval: int,
        max_fps: float,
        log_info: Callable[[str], None],
        log_warning: Callable[[str], None],
        source_status: Optional[Callable[[], dict]] = None,
        select_source: Optional[Callable[[str], dict]] = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.bitrate = int(bitrate)
        self.iframe_interval = max(1, int(iframe_interval))
        self.max_fps = max(1.0, float(max_fps))
        self._log_info = log_info
        self._log_warning = log_warning
        self._source_status = source_status
        self._select_source = select_source

        self._peer_lock = threading.RLock()
        self._peers: dict[str, _Peer] = {}
        self._orphaned_peer_branches: list[tuple[object, object]] = []
        self._frame_lock = threading.Lock()
        self._frame_sequence = 0
        self._next_stamp_ns: int | None = None
        self._last_input_stamp_ns: int | None = None
        self._last_pts_ns = -1
        self._accepted_frame_stamps_ns: deque[int] = deque(maxlen=120)

        self._gst_loop = None
        self._gst_thread: threading.Thread | None = None
        self._async_loop: asyncio.AbstractEventLoop | None = None
        self._async_thread: threading.Thread | None = None
        self._shutdown_event = None
        self._ready = threading.Event()
        self._stopped = threading.Event()
        self._startup_error = ""

        self._load_gstreamer()
        self._create_pipeline()

    def _load_gstreamer(self) -> None:
        import gi

        gi.require_version("Gst", "1.0")
        gi.require_version("GstSdp", "1.0")
        gi.require_version("GstWebRTC", "1.0")
        from gi.repository import GLib, Gst, GstSdp, GstWebRTC

        Gst.init(None)
        local_plugin_path = os.environ.get(
            "YOLO_GST_PLUGIN_PATH",
            os.path.expanduser("~/.local/lib/gstreamer-1.0"),
        )
        if os.path.isdir(local_plugin_path):
            Gst.Registry.get().scan_path(local_plugin_path)
        self.GLib = GLib
        self.Gst = Gst
        self.GstSdp = GstSdp
        self.GstWebRTC = GstWebRTC
        if Gst.ElementFactory.find("nicesrc") is None:
            raise RuntimeError(
                "GStreamer libnice plugin missing; install gstreamer1.0-nice"
            )

    def _create_pipeline(self) -> None:
        description = (
            "appsrc name=video_source is-live=true block=false format=time "
            "do-timestamp=false caps=image/jpeg "
            "! queue leaky=downstream max-size-buffers=2 "
            "! nvjpegdec "
            "! nvvidconv "
            "! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2h264enc name=video_encoder bitrate={self.bitrate} "
            f"iframeinterval={self.iframe_interval} "
            f"idrinterval={self.iframe_interval} profile=1 "
            "insert-sps-pps=true maxperf-enable=true copy-timestamp=true "
            "! h264parse config-interval=-1 "
            "! video/x-h264,stream-format=byte-stream,alignment=au "
            "! rtph264pay name=video_payloader pt=96 config-interval=-1 "
            "timestamp-offset=0 perfect-rtptime=true aggregate-mode=zero-latency "
            "! application/x-rtp,media=video,encoding-name=H264,"
            "payload=96,clock-rate=90000,packetization-mode=(string)1,"
            "profile-level-id=(string)42e01f "
            "! tee name=rtp_tee allow-not-linked=true"
        )
        self.pipeline = self.Gst.parse_launch(description)
        self.appsrc = self.pipeline.get_by_name("video_source")
        self.encoder = self.pipeline.get_by_name("video_encoder")
        self.tee = self.pipeline.get_by_name("rtp_tee")
        if self.appsrc is None or self.encoder is None or self.tee is None:
            raise RuntimeError("failed to create WebRTC GStreamer elements")

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    @property
    def startup_error(self) -> str:
        return self._startup_error

    @property
    def peer_count(self) -> int:
        with self._peer_lock:
            return len(self._peers)

    def reset_source_timing(self) -> None:
        with self._frame_lock:
            self._next_stamp_ns = None
            self._last_input_stamp_ns = None
            self._accepted_frame_stamps_ns.clear()

    @property
    def encoded_fps(self) -> float:
        with self._frame_lock:
            if len(self._accepted_frame_stamps_ns) < 2:
                return 0.0
            latest = self._accepted_frame_stamps_ns[-1]
            cutoff = latest - 2_000_000_000
            recent = [
                stamp
                for stamp in self._accepted_frame_stamps_ns
                if stamp >= cutoff
            ]
        if len(recent) < 2 or recent[-1] <= recent[0]:
            return 0.0
        return (len(recent) - 1) * 1_000_000_000 / (recent[-1] - recent[0])

    def start(self, timeout: float = 5.0) -> bool:
        if self._async_thread is not None:
            return not self._startup_error

        self._gst_loop = self.GLib.MainLoop()
        self._gst_thread = threading.Thread(
            target=self._gst_loop.run,
            name="yolo-webrtc-gstreamer",
            daemon=True,
        )
        self._gst_thread.start()

        state_result = self.pipeline.set_state(self.Gst.State.PLAYING)
        if state_result == self.Gst.StateChangeReturn.FAILURE:
            self._startup_error = "GStreamer pipeline failed to enter PLAYING"
            self.stop()
            return False

        self._async_thread = threading.Thread(
            target=self._run_async_server,
            name="yolo-webrtc-signaling",
            daemon=True,
        )
        self._async_thread.start()
        if not self._ready.wait(timeout):
            self._startup_error = "WebRTC signaling server startup timed out"
            self.stop()
            return False
        if self._startup_error:
            self.stop()
            return False

        self._log_info(
            f"WebRTC H.264 signaling ws://{self.host}:{self.port} "
            f"bitrate={self.bitrate} max_fps={self.max_fps:.1f}"
        )
        return True

    def stop(self) -> None:
        loop = self._async_loop
        event = self._shutdown_event
        if loop is not None and event is not None and loop.is_running():
            loop.call_soon_threadsafe(event.set)
        if self._async_thread is not None and self._async_thread.is_alive():
            self._async_thread.join(timeout=3.0)
        self._async_thread = None

        try:
            self.appsrc.emit("end-of-stream")
            self.pipeline.set_state(self.Gst.State.NULL)
        except Exception:
            pass
        if self._gst_loop is not None:
            self._gst_loop.quit()
        if self._gst_thread is not None and self._gst_thread.is_alive():
            self._gst_thread.join(timeout=2.0)
        self._gst_thread = None
        self._stopped.set()

    def push_frame(
        self,
        jpeg_data: bytes,
        stamp_sec: int,
        stamp_nanosec: int,
        width: int,
        height: int,
        detections: list[dict],
    ) -> bool:
        if self.peer_count <= 0 or self._startup_error:
            return False

        stamp_ns = int(stamp_sec) * 1_000_000_000 + int(stamp_nanosec)
        with self._frame_lock:
            min_interval_ns = int(1_000_000_000 / self.max_fps)
            if (
                self._last_input_stamp_ns is not None
                and stamp_ns < self._last_input_stamp_ns
            ):
                self._next_stamp_ns = None
            self._last_input_stamp_ns = stamp_ns
            if self._next_stamp_ns is not None:
                if stamp_ns < self._next_stamp_ns:
                    return False
                intervals = ((stamp_ns - self._next_stamp_ns) // min_interval_ns) + 1
                self._next_stamp_ns += intervals * min_interval_ns
            else:
                self._next_stamp_ns = stamp_ns + min_interval_ns
            # appsrc is live, so PTS must be in the pipeline's running-time
            # domain.  ROS/header time is deliberately retained only in the
            # side-band metadata used for bbox matching.
            clock = self.pipeline.get_clock()
            if clock is None:
                self._log_warning("WebRTC pipeline clock is unavailable")
                return False
            pts_ns = max(
                0,
                int(clock.get_time()) - int(self.pipeline.get_base_time()),
            )
            if pts_ns <= self._last_pts_ns:
                pts_ns = self._last_pts_ns + 1
            self._last_pts_ns = pts_ns
            self._frame_sequence += 1
            frame_sequence = self._frame_sequence
            self._accepted_frame_stamps_ns.append(stamp_ns)

        buffer = self.Gst.Buffer.new_allocate(None, len(jpeg_data), None)
        buffer.fill(0, jpeg_data)
        buffer.pts = pts_ns
        buffer.dts = pts_ns
        buffer.duration = int(1_000_000_000 / self.max_fps)
        result = self.appsrc.emit("push-buffer", buffer)
        if result != self.Gst.FlowReturn.OK:
            self._log_warning(f"WebRTC appsrc push failed: {result.value_nick}")
            return False

        rtp_timestamp = (
            pts_ns * self.RTP_CLOCK_RATE // 1_000_000_000
        ) & 0xFFFFFFFF
        self._broadcast(
            {
                "type": "frame",
                "frame": {
                    "sequence": frame_sequence,
                    "stamp": {
                        "sec": int(stamp_sec),
                        "nanosec": int(stamp_nanosec),
                    },
                    "pts_ns": str(pts_ns),
                    "rtp_timestamp": rtp_timestamp,
                    "width": int(width),
                    "height": int(height),
                    "detections": detections,
                },
            },
            drop_if_full=True,
        )
        return True

    def _run_async_server(self) -> None:
        try:
            asyncio.run(self._async_server_main())
        except Exception as exc:
            self._startup_error = f"WebRTC signaling failed: {exc}"
            self._log_warning(self._startup_error)
            self._ready.set()
        finally:
            self._stopped.set()

    async def _async_server_main(self) -> None:
        from websockets.server import serve

        self._async_loop = asyncio.get_running_loop()
        self._shutdown_event = asyncio.Event()
        async with serve(
            self._handle_websocket,
            self.host,
            self.port,
            max_size=2 * 1024 * 1024,
            ping_interval=10,
            ping_timeout=10,
        ):
            self._ready.set()
            await self._shutdown_event.wait()

    async def _handle_websocket(self, websocket) -> None:
        request = getattr(websocket, "request", None)
        request_path = str(getattr(request, "path", ""))
        control_only = request_path.partition("?")[0].rstrip("/").endswith(
            "/control"
        )
        peer = _Peer(
            peer_id=uuid.uuid4().hex[:10],
            websocket=websocket,
            outgoing=asyncio.Queue(maxsize=64),
        )
        sender = asyncio.create_task(self._peer_sender(peer))
        peer_added = False
        try:
            if not control_only:
                await asyncio.to_thread(self._add_peer_sync, peer)
                peer_added = True
            await peer.outgoing.put(
                json.dumps(
                    {
                        "type": "ready",
                        "transport": "webrtc_h264",
                        "peer_id": peer.peer_id,
                        "source": self._current_source_status(),
                    },
                    separators=(",", ":"),
                )
            )
            async for raw_message in websocket:
                if not isinstance(raw_message, str):
                    continue
                try:
                    message = json.loads(raw_message)
                except json.JSONDecodeError:
                    continue
                self._handle_peer_message(peer, message)
        except Exception as exc:
            self._log_warning(f"WebRTC peer {peer.peer_id} closed: {exc}")
        finally:
            if peer_added:
                await asyncio.to_thread(self._remove_peer_sync, peer)
            sender.cancel()
            try:
                await sender
            except asyncio.CancelledError:
                pass
            except Exception:
                pass

    async def _peer_sender(self, peer: _Peer) -> None:
        while True:
            message = await peer.outgoing.get()
            await peer.websocket.send(message)

    def _add_peer_sync(self, peer: _Peer) -> None:
        def add_peer() -> None:
            queue = self.Gst.ElementFactory.make(
                "queue", f"webrtc_queue_{peer.peer_id}"
            )
            webrtc = self.Gst.ElementFactory.make(
                "webrtcbin", f"webrtc_{peer.peer_id}"
            )
            if queue is None or webrtc is None:
                raise RuntimeError("failed to create WebRTC peer elements")
            queue.set_property("leaky", 2)
            # This queue contains RTP packets, not video frames.  A 720p IDR is
            # split across many packets, so a four-buffer queue can discard most
            # of the keyframe while ICE/DTLS is becoming ready.
            queue.set_property("max-size-buffers", 256)
            queue.set_property("max-size-bytes", 0)
            queue.set_property("max-size-time", 1_000_000_000)
            webrtc.set_property("bundle-policy", "max-bundle")
            webrtc.set_property("latency", 50)
            peer.signal_handlers = [
                webrtc.connect(
                    "on-negotiation-needed", self._on_negotiation_needed, peer
                ),
                webrtc.connect("on-ice-candidate", self._on_ice_candidate, peer),
                webrtc.connect(
                    "notify::ice-connection-state", self._on_peer_state, peer
                ),
                webrtc.connect(
                    "notify::connection-state", self._on_peer_state, peer
                ),
            ]

            self.pipeline.add(queue)
            self.pipeline.add(webrtc)
            tee_pad = self.tee.request_pad_simple("src_%u")
            webrtc_pad = webrtc.request_pad_simple("sink_%u")
            if tee_pad is None or webrtc_pad is None:
                raise RuntimeError("failed to request WebRTC RTP pads")
            transceiver = webrtc_pad.get_property("transceiver")
            if transceiver is not None:
                transceiver.set_property(
                    "direction",
                    self.GstWebRTC.WebRTCRTPTransceiverDirection.SENDONLY,
                )
            if tee_pad.link(queue.get_static_pad("sink")) != self.Gst.PadLinkReturn.OK:
                raise RuntimeError("failed to link RTP tee to peer queue")
            if (
                queue.get_static_pad("src").link(webrtc_pad)
                != self.Gst.PadLinkReturn.OK
            ):
                raise RuntimeError("failed to link peer queue to webrtcbin")

            peer.queue = queue
            peer.webrtc = webrtc
            peer.tee_pad = tee_pad
            peer.webrtc_pad = webrtc_pad
            peer.pending_ice = []
            with self._peer_lock:
                self._peers[peer.peer_id] = peer
            queue.sync_state_with_parent()
            webrtc.sync_state_with_parent()
            self._on_negotiation_needed(webrtc, peer)

        self._gst_call_sync(add_peer)
        self._log_info(f"WebRTC peer connected: {peer.peer_id}")

    def _remove_peer_sync(self, peer: _Peer) -> None:
        with self._peer_lock:
            existed = self._peers.pop(peer.peer_id, None) is not None
        if not existed:
            return

        def remove_peer() -> None:
            queue = peer.queue
            webrtc = peer.webrtc
            tee_pad = peer.tee_pad
            webrtc_pad = peer.webrtc_pad
            if webrtc is not None:
                for handler_id in peer.signal_handlers or []:
                    if webrtc.handler_is_connected(handler_id):
                        webrtc.disconnect(handler_id)
            # Detach the live tee before stopping the branch.  Stopping a queue
            # while the tee is still pushing into it can deadlock the GLib main
            # context and prevent subsequent peers from being added.
            if tee_pad is not None and queue is not None:
                tee_pad.unlink(queue.get_static_pad("sink"))
                self.tee.release_request_pad(tee_pad)
            if not peer.remote_description_set:
                # Tearing a webrtcbin in HAVE_LOCAL_OFFER down can block the
                # GStreamer main context on Jetson's GStreamer 1.20 stack.  The
                # branch is already detached and receives no more RTP; retain
                # its elements until the parent pipeline is stopped instead of
                # taking the whole transport down with a half-negotiated peer.
                if queue is not None and webrtc is not None:
                    self._orphaned_peer_branches.append((queue, webrtc))
                peer.queue = None
                peer.webrtc = None
                peer.tee_pad = None
                peer.webrtc_pad = None
                peer.signal_handlers = None
                peer.remote_answer = None
                peer.remote_answer_promise = None
                return
            if queue is not None and webrtc is not None and webrtc_pad is not None:
                queue.get_static_pad("src").unlink(webrtc_pad)
                webrtc.release_request_pad(webrtc_pad)
            for element in (queue, webrtc):
                if element is not None:
                    element.set_state(self.Gst.State.NULL)
            for element in (queue, webrtc):
                if element is not None:
                    self.pipeline.remove(element)
            peer.queue = None
            peer.webrtc = None
            peer.tee_pad = None
            peer.webrtc_pad = None
            peer.signal_handlers = None
            peer.remote_answer = None
            peer.remote_answer_promise = None

        try:
            self._gst_call_sync(remove_peer)
        except Exception as exc:
            self._log_warning(f"WebRTC peer cleanup failed: {exc}")
        self._log_info(f"WebRTC peer disconnected: {peer.peer_id}")

    def _gst_call_sync(self, callback: Callable[[], None], timeout: float = 5.0) -> None:
        completed = threading.Event()
        result: dict[str, Exception] = {}

        def invoke() -> bool:
            try:
                callback()
            except Exception as exc:
                result["error"] = exc
            finally:
                completed.set()
            return False

        self.GLib.idle_add(invoke)
        if not completed.wait(timeout):
            raise RuntimeError("timed out while updating WebRTC pipeline")
        if "error" in result:
            raise result["error"]

    def _gst_call_async(self, callback: Callable[[], None]) -> None:
        def invoke() -> bool:
            try:
                callback()
            except Exception as exc:
                self._log_warning(f"WebRTC GStreamer callback failed: {exc}")
            return False

        self.GLib.idle_add(invoke)

    def _on_negotiation_needed(self, webrtc, peer: _Peer) -> None:
        if peer.offer_started:
            return
        peer.offer_started = True
        promise = self.Gst.Promise.new_with_change_func(
            self._on_offer_created,
            peer,
            None,
        )
        webrtc.emit("create-offer", None, promise)

    def _on_offer_created(self, promise, peer: _Peer, _unused) -> None:
        reply = promise.get_reply()
        offer = reply.get_value("offer") if reply is not None else None
        if offer is None or peer.webrtc is None:
            self._queue_peer(
                peer,
                {"type": "error", "message": "failed to create WebRTC offer"},
            )
            return
        peer.local_offer_sdp = offer.sdp.as_text()
        local_description_promise = self.Gst.Promise.new_with_change_func(
            self._on_local_description_set,
            peer,
            None,
        )
        peer.webrtc.emit(
            "set-local-description",
            offer,
            local_description_promise,
        )

    def _on_local_description_set(self, promise, peer: _Peer, _unused) -> None:
        reply = promise.get_reply()
        error = reply.get_value("error") if reply is not None else None
        offer_sdp = peer.local_offer_sdp
        if error is not None or not offer_sdp:
            self._queue_peer(
                peer,
                {
                    "type": "error",
                    "message": f"failed to set local WebRTC offer: {error}",
                },
            )
            return
        signaling_state = peer.webrtc.get_property("signaling-state")
        self._log_info(
            f"WebRTC peer {peer.peer_id} local offer ready: "
            f"signaling={signaling_state.value_nick}"
        )
        self._queue_peer(
            peer,
            {
                "type": "sdp",
                "sdp": {"type": "offer", "sdp": offer_sdp},
            },
        )

    def _on_ice_candidate(
        self,
        _webrtc,
        mline_index: int,
        candidate: str,
        peer: _Peer,
    ) -> None:
        self._queue_peer(
            peer,
            {
                "type": "ice",
                "ice": {
                    "candidate": candidate,
                    "sdpMLineIndex": int(mline_index),
                },
            },
        )

    def _handle_peer_message(self, peer: _Peer, message: dict) -> None:
        message_type = str(message.get("type", ""))
        if message_type == "source_list":
            self._queue_peer(
                peer,
                {"type": "sources", **self._current_source_status()},
            )
        elif message_type == "select_source":
            topic = str(message.get("topic", "")).strip()
            if self._select_source is None:
                self._queue_peer(
                    peer,
                    {"type": "source_error", "message": "source selection is unavailable"},
                )
                return
            try:
                result = self._select_source(topic)
                self._queue_peer(peer, {"type": "source_selection", **result})
            except Exception as exc:
                self._queue_peer(
                    peer,
                    {"type": "source_error", "message": str(exc)},
                )
        elif peer.webrtc is None:
            return
        elif message_type == "sdp":
            sdp_data = message.get("sdp") or {}
            if sdp_data.get("type") != "answer":
                return
            answer_sdp = str(sdp_data.get("sdp", ""))
            video_lines = [
                line.strip()
                for line in answer_sdp.splitlines()
                if line.startswith("m=video ")
            ]
            video_fields = video_lines[0].split() if video_lines else []
            if len(video_fields) < 2 or video_fields[1] == "0":
                self._queue_peer(
                    peer,
                    {
                        "type": "error",
                        "message": "browser rejected the offered H.264 video profile",
                    },
                )
                return
            has_ice_ufrag = any(
                line.startswith("a=ice-ufrag:") and line.partition(":")[2].strip()
                for line in answer_sdp.splitlines()
            )
            has_ice_pwd = any(
                line.startswith("a=ice-pwd:") and line.partition(":")[2].strip()
                for line in answer_sdp.splitlines()
            )
            if not has_ice_ufrag or not has_ice_pwd:
                # Some browsers briefly expose an incomplete local description.
                # Never pass that SDP to webrtcbin: affected GStreamer versions can
                # block the GLib thread while trying to apply it.
                self._queue_peer(
                    peer,
                    {
                        "type": "error",
                        "message": "browser SDP answer is missing ICE credentials",
                    },
                )
                return
            result, sdp_message = self.GstSdp.SDPMessage.new()
            if result != self.GstSdp.SDPResult.OK:
                raise RuntimeError("failed to allocate SDP message")
            parse_result = self.GstSdp.sdp_message_parse_buffer(
                answer_sdp.encode("utf-8"),
                sdp_message,
            )
            if parse_result != self.GstSdp.SDPResult.OK:
                raise RuntimeError("failed to parse browser SDP answer")
            answer = self.GstWebRTC.WebRTCSessionDescription.new(
                self.GstWebRTC.WebRTCSDPType.ANSWER,
                sdp_message,
            )
            promise = self.Gst.Promise.new_with_change_func(
                self._on_remote_description_set,
                peer,
                None,
            )
            peer.remote_answer = answer
            peer.remote_answer_promise = promise

            def set_remote_description() -> None:
                if peer.webrtc is None:
                    return
                signaling_state = peer.webrtc.get_property("signaling-state")
                self._log_info(
                    f"WebRTC peer {peer.peer_id} received browser answer: "
                    f"signaling={signaling_state.value_nick}"
                )
                peer.webrtc.emit(
                    "set-remote-description",
                    answer,
                    promise,
                )

            self._gst_call_async(set_remote_description)
        elif message_type == "ice":
            ice = message.get("ice") or {}
            candidate = str(ice.get("candidate", ""))
            if candidate:
                mline_index = int(ice.get("sdpMLineIndex", 0))
                with self._peer_lock:
                    remote_description_set = peer.remote_description_set
                    if not remote_description_set and peer.pending_ice is not None:
                        peer.pending_ice.append((mline_index, candidate))
                if remote_description_set:
                    self._gst_call_async(
                        lambda: peer.webrtc.emit(
                            "add-ice-candidate",
                            mline_index,
                            candidate,
                        )
                    )

    def _current_source_status(self) -> dict:
        if self._source_status is None:
            return {"selected": "", "mode": "video_only", "sources": []}
        try:
            status = self._source_status()
            return dict(status) if isinstance(status, dict) else {}
        except Exception as exc:
            self._log_warning(f"WebRTC source status failed: {exc}")
            return {
                "selected": "",
                "mode": "video_only",
                "sources": [],
                "error": str(exc),
            }

    def broadcast_source_status(self) -> None:
        self._broadcast({"type": "sources", **self._current_source_status()})

    def _on_remote_description_set(self, promise, peer: _Peer, _unused) -> None:
        reply = promise.get_reply()
        error = reply.get_value("error") if reply is not None else None
        if error is not None or peer.webrtc is None:
            self._queue_peer(
                peer,
                {
                    "type": "error",
                    "message": f"failed to set browser SDP answer: {error}",
                },
            )
            return
        with self._peer_lock:
            peer.remote_description_set = True
            peer.remote_answer = None
            peer.remote_answer_promise = None
            pending_ice = list(peer.pending_ice or [])
            if peer.pending_ice is not None:
                peer.pending_ice.clear()
        self._log_info(f"WebRTC peer {peer.peer_id} accepted browser SDP answer")
        for mline_index, candidate in pending_ice:
            self._gst_call_async(
                lambda index=mline_index, value=candidate: peer.webrtc.emit(
                    "add-ice-candidate",
                    index,
                    value,
                )
            )

    def _on_peer_state(self, webrtc, _property, peer: _Peer) -> None:
        ice_state = webrtc.get_property("ice-connection-state")
        connection_state = webrtc.get_property("connection-state")
        self._log_info(
            f"WebRTC peer {peer.peer_id} state: "
            f"ice={ice_state.value_nick}, connection={connection_state.value_nick}"
        )

    def _queue_peer(self, peer: _Peer, payload: dict, drop_if_full: bool = False) -> None:
        loop = self._async_loop
        if loop is None or not loop.is_running():
            return
        encoded = json.dumps(payload, separators=(",", ":"))

        def enqueue() -> None:
            if drop_if_full and peer.outgoing.full():
                return
            try:
                peer.outgoing.put_nowait(encoded)
            except asyncio.QueueFull:
                pass

        loop.call_soon_threadsafe(enqueue)

    def _broadcast(self, payload: dict, drop_if_full: bool = False) -> None:
        with self._peer_lock:
            peers = list(self._peers.values())
        for peer in peers:
            self._queue_peer(peer, payload, drop_if_full=drop_if_full)

    def _on_bus_message(self, _bus, message) -> None:
        if message.type == self.Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            self._startup_error = str(error)
            self._log_warning(
                f"WebRTC GStreamer error: {error}; debug={debug or '--'}"
            )
            self._broadcast(
                {"type": "error", "message": f"GStreamer: {error}"}
            )
        elif message.type == self.Gst.MessageType.WARNING:
            warning, debug = message.parse_warning()
            self._log_warning(
                f"WebRTC GStreamer warning: {warning}; debug={debug or '--'}"
            )

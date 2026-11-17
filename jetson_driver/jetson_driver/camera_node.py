"""Camera node — publishes sensor_msgs/Image on /front_camera (and, if a
second camera is configured, /rear_camera).

Supports three source modes via the `source` parameter:
  * 'csi'  — Jetson CSI camera via nvarguscamerasrc GStreamer pipeline.
             Lowest latency on the Orin Nano because frames stay in NVMM
             until the final BGR conversion.
  * 'usb'  — UVC camera at /dev/videoN (index) or /dev/video_FRONT (path).
  * 'file' — mp4 / avi path, useful for desk testing without cameras.

Each camera runs in its own daemon thread.  cv2.VideoCapture.read() is a
blocking call (it waits for the next frame from the device), so a single
timer reading both cameras serially would cap the combined rate at the
slower camera's interval.  Per-camera threads keep each stream at its
native rate.  rclpy publishers are thread-safe so no extra locking is
required.

QoS = SensorDataQoS (best-effort, depth 1) — high-rate imagery; drop a
frame rather than retransmit.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from .common import CAM_W, CAM_H, CAMERA_HZ


def _gst_pipeline_csi(sensor_id: int, width: int, height: int,
                      fps: int) -> str:
    """GStreamer pipeline for a Jetson CSI camera (IMX219, IMX477...)."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink drop=true max-buffers=1"
    )


def _device_enabled(device) -> bool:
    """True if the device parameter selects a real camera.

    Accepts integer indices, integer-as-string indices, paths, or the
    sentinel values None / "" / "-1" / -1 (which mean "this slot is unused")."""
    if device is None:
        return False
    if isinstance(device, int):
        return device >= 0
    s = str(device).strip()
    if s == "" or s == "-1":
        return False
    return True


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # Devices are strings so they accept either a V4L2 index ("0", "1")
        # or a /dev path ("/dev/video_FRONT").  "-1" or "" disables a slot.
        self.declare_parameter('source',          'usb')
        self.declare_parameter('front_device',    '0')
        self.declare_parameter('rear_device',     '-1')
        self.declare_parameter('width',           CAM_W)
        self.declare_parameter('height',          CAM_H)
        self.declare_parameter('fps',             CAMERA_HZ)
        self.declare_parameter('flip_front',      False)
        self.declare_parameter('flip_rear',       True)

        self.source       = self.get_parameter('source').value
        self.front_device = self.get_parameter('front_device').value
        self.rear_device  = self.get_parameter('rear_device').value
        self.width        = int(self.get_parameter('width').value)
        self.height       = int(self.get_parameter('height').value)
        self.fps          = int(self.get_parameter('fps').value)
        self.flip_front   = bool(self.get_parameter('flip_front').value)
        self.flip_rear    = bool(self.get_parameter('flip_rear').value)

        # publishers
        self.front_pub = self.create_publisher(
            Image, '/front_camera', qos_profile_sensor_data)
        self.rear_pub = None
        if _device_enabled(self.rear_device):
            self.rear_pub = self.create_publisher(
                Image, '/rear_camera', qos_profile_sensor_data)

        self.bridge = CvBridge()
        self.cap_front = self._open(self.front_device)
        self.cap_rear  = self._open(self.rear_device) if self.rear_pub else None

        if self.cap_front is None or not self.cap_front.isOpened():
            self.get_logger().error(
                f"Could not open front camera (source={self.source}, "
                f"device={self.front_device}).")
        if self.rear_pub is not None and (
                self.cap_rear is None or not self.cap_rear.isOpened()):
            self.get_logger().error(
                f"Could not open rear camera (source={self.source}, "
                f"device={self.rear_device}).")

        # One thread per camera so each can publish at its native rate
        # without blocking the other.  Daemon threads die with the process.
        self._stop_event = threading.Event()
        self._threads = []
        if self.cap_front is not None and self.cap_front.isOpened():
            self._threads.append(self._spawn_capture_thread(
                'cam_front', self.cap_front, self.front_pub,
                'front_cam', self.flip_front))
        if self.cap_rear is not None and self.cap_rear.isOpened():
            self._threads.append(self._spawn_capture_thread(
                'cam_rear', self.cap_rear, self.rear_pub,
                'rear_cam', self.flip_rear))

        self.get_logger().info(
            f"camera_node up: source={self.source} front={self.front_device} "
            f"rear={self.rear_device if self.rear_pub else 'off'} "
            f"{self.width}x{self.height}@{self.fps}Hz")

    # ------------------------------------------------------------------
    def _open(self, device):
        if not _device_enabled(device):
            return None

        if self.source == 'csi':
            sid = int(device) if isinstance(device, int) else 0
            pipeline = _gst_pipeline_csi(sid, self.width, self.height,
                                         self.fps)
            return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if self.source == 'file':
            return cv2.VideoCapture(str(device))

        # 'usb' — accept integer index or /dev path.  Path form lets udev
        # symlinks pin front/rear assignment across reboots.
        if isinstance(device, str) and not device.lstrip('-').isdigit():
            cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        else:
            cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
        # MJPG before width/height/fps — UVC cams default to YUYV which
        # saturates USB 2.0 bandwidth at ~7-10 fps; MJPG hits the full 30.
        cap.set(cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS,          self.fps)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap

    # ------------------------------------------------------------------
    def _spawn_capture_thread(self, name, cap, pub, frame_id, flip):
        t = threading.Thread(
            target=self._capture_loop,
            args=(cap, pub, frame_id, flip),
            name=name,
            daemon=True,
        )
        t.start()
        return t

    # ------------------------------------------------------------------
    def _capture_loop(self, cap, pub, frame_id: str, flip: bool):
        """Tight per-camera loop: read → flip → publish until shutdown."""
        consecutive_failures = 0
        while not self._stop_event.is_set():
            try:
                if not cap.isOpened():
                    break
                ok, frame = cap.read()
            except Exception:
                ok, frame = False, None

            if not ok or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= 30:
                    self.get_logger().error(
                        f"{frame_id}: 30 consecutive read failures, "
                        f"giving up on this camera.")
                    return
                self._stop_event.wait(0.05)
                continue
            consecutive_failures = 0

            if flip:
                frame = cv2.flip(frame, 1)
            try:
                self._publish(pub, frame, frame_id)
            except Exception:
                return  # rclpy shutdown — exit thread

    # ------------------------------------------------------------------
    def _publish(self, pub, frame, frame_id: str):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        # Stop threads BEFORE releasing captures so they don't touch a
        # freed device.
        try:
            self._stop_event.set()
        except Exception:
            pass
        for t in getattr(self, '_threads', []):
            try:
                t.join(timeout=2.0)
            except Exception:
                pass
        for cap_attr in ('cap_front', 'cap_rear'):
            try:
                cap = getattr(self, cap_attr, None)
                if cap is not None:
                    cap.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

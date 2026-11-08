#!/usr/bin/env python3

import argparse
import base64
import json
import math
import socketserver
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header, String
from visualization_msgs.msg import Marker, MarkerArray


MAX_LINE_BYTES = 1_000_000
MAX_PATH_POSES = 3000

# ORB-SLAM3 tracking states: name and marker color (r, g, b). Poses in states other than
# OK are IMU predictions (or the last failed estimate) and are shown in warning colors.
STATE_STYLE = {
    2: ("OK", (0.1, 0.9, 0.1)),
    3: ("RECENTLY_LOST (IMU prediction)", (1.0, 0.6, 0.0)),
    4: ("LOST", (1.0, 0.1, 0.1)),
    5: ("OK_KLT", (0.1, 0.9, 0.1)),
}


class LatestFrameQueue:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._images = None
        self._received = 0
        self._invalid = 0
        self._last_tracking_state = None
        self._last_point_count = 0

    def put(self, frame):
        with self._lock:
            self._received += 1
            if frame.get("type") == "stereo_image":
                self._images = frame
                return
            self._frame = frame
            self._last_tracking_state = frame["tracking_state"]
            self._last_point_count = len(frame["points"])

    def reject(self):
        with self._lock:
            self._invalid += 1

    def get(self):
        with self._lock:
            frame = self._frame
            images = self._images
            self._frame = None
            self._images = None
            return frame, images

    def stats(self):
        with self._lock:
            return (
                self._received,
                self._invalid,
                self._last_tracking_state,
                self._last_point_count,
            )


class FrameHandler(socketserver.StreamRequestHandler):
    def handle(self):
        while True:
            raw = self.rfile.readline(MAX_LINE_BYTES + 1)
            if not raw:
                return
            if len(raw) > MAX_LINE_BYTES or not raw.endswith(b"\n"):
                self.server.frame_queue.reject()
                return
            try:
                frame = json.loads(raw)
                validate_frame(frame)
            except (json.JSONDecodeError, TypeError, ValueError, KeyError):
                self.server.frame_queue.reject()
                continue
            self.server.frame_queue.put(frame)


def finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def validate_frame(frame):
    if not isinstance(frame, dict):
        raise TypeError("frame must be a JSON object")
    if frame.get("type") == "stereo_image":
        width = frame.get("image_width")
        height = frame.get("image_height")
        image_fields = ("left_image", "right_image")
        if (
            type(width) is not int
            or type(height) is not int
            or not 1 <= width <= 640
            or not 1 <= height <= 480
            or not all(field in frame for field in image_fields)
        ):
            raise ValueError("invalid image dimensions or missing stereo image")
        for field in image_fields:
            encoded = frame[field]
            if not isinstance(encoded, str):
                raise ValueError("invalid encoded image")
            try:
                image_data = base64.b64decode(encoded, validate=True)
            except ValueError as error:
                raise ValueError("invalid base64 image data") from error
            if len(image_data) != width * height:
                raise ValueError("image data size does not match dimensions")
            frame[field] = image_data
        return

    if not finite_number(frame["timestamp"]):
        raise ValueError("invalid timestamp")
    if not isinstance(frame["tracking_state"], int):
        raise ValueError("invalid tracking state")
    if len(frame["position"]) != 3 or not all(finite_number(v) for v in frame["position"]):
        raise ValueError("invalid position")
    if len(frame["orientation"]) != 4 or not all(
        finite_number(v) for v in frame["orientation"]
    ):
        raise ValueError("invalid orientation")
    if not isinstance(frame["points"], list):
        raise ValueError("invalid points")

class FrameServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, frame_queue):
        self.frame_queue = frame_queue
        super().__init__(address, FrameHandler)


class RvizBridge(Node):
    def __init__(self, frame_queue):
        super().__init__("orb_slam_tcp_bridge")
        self._frame_queue = frame_queue
        self._path = Path()
        self._path.header.frame_id = "map"
        self._published = 0
        self._image_frames_published = 0
        self._last_status_log = time.monotonic()
        self._left_image_pub = self.create_publisher(
            Image, "/orb_slam/camera_left/image_raw", qos_profile_sensor_data
        )
        self._right_image_pub = self.create_publisher(
            Image, "/orb_slam/camera_right/image_raw", qos_profile_sensor_data
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/orb_slam/pose", qos_profile_sensor_data
        )
        self._path_pub = self.create_publisher(Path, "/orb_slam/path", 1)
        self._lost_path = Path()
        self._lost_path.header.frame_id = "map"
        self._lost_path_pub = self.create_publisher(Path, "/orb_slam/path_lost", 1)
        self._state_pub = self.create_publisher(String, "/orb_slam/tracking_state", 1)
        self._marker_pub = self.create_publisher(MarkerArray, "/orb_slam/state_marker", 1)
        self._cloud_pub = self.create_publisher(
            PointCloud2, "/orb_slam/map_points", qos_profile_sensor_data
        )
        self._timer = self.create_timer(0.02, self._publish_latest)

    def _publish_state_marker(self, header, pose, state_name, color):
        rgba = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
        sphere = Marker()
        sphere.header = header
        sphere.ns = "tracking_state"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = pose.pose
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
        sphere.color = rgba

        text = Marker()
        text.header = header
        text.ns = "tracking_state"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.pose.position.x
        text.pose.position.y = pose.pose.position.y
        text.pose.position.z = pose.pose.position.z + 0.25
        text.pose.orientation.w = 1.0
        text.scale.z = 0.12
        text.color = rgba
        text.text = state_name

        self._marker_pub.publish(MarkerArray(markers=[sphere, text]))

    def _publish_latest(self):
        frame, images = self._frame_queue.get()
        if images is not None:
            stamp = self.get_clock().now().to_msg()
            for publisher, field, frame_id in (
                (self._left_image_pub, "left_image", "camera_left"),
                (self._right_image_pub, "right_image", "camera_right"),
            ):
                image = Image()
                image.header = Header()
                image.header.stamp = stamp
                image.header.frame_id = frame_id
                image.height = images["image_height"]
                image.width = images["image_width"]
                image.encoding = "mono8"
                image.is_bigendian = 0
                image.step = image.width
                image.data = images[field]
                publisher.publish(image)
            self._image_frames_published += 1

        if frame is not None:
            stamp = self.get_clock().now().to_msg()
            header = Header()
            header.stamp = stamp
            state = frame["tracking_state"]
            if state in STATE_STYLE and frame.get("pose_valid", True):
                state_name, color = STATE_STYLE[state]
                tracked = state in (2, 5)
                header.frame_id = "map"
                pose = PoseStamped()
                pose.header = header
                (
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ) = (float(value) for value in frame["position"])
                (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ) = (float(value) for value in frame["orientation"])
                self._pose_pub.publish(pose)

                path = self._path if tracked else self._lost_path
                path.header = header
                path.poses.append(pose)
                if len(path.poses) > MAX_PATH_POSES:
                    del path.poses[: len(path.poses) - MAX_PATH_POSES]
                (self._path_pub if tracked else self._lost_path_pub).publish(path)

                self._state_pub.publish(String(data=state_name))
                self._publish_state_marker(header, pose, state_name, color)

                points = [
                    tuple(float(value) for value in point)
                    for point in frame["points"]
                    if isinstance(point, list)
                    and len(point) == 3
                    and all(finite_number(value) for value in point)
                ]
                cloud = point_cloud2.create_cloud_xyz32(header, points)
                self._cloud_pub.publish(cloud)
                self._published += 1
            else:
                self._state_pub.publish(String(data=f"NO_POSE (state {state})"))

        now = time.monotonic()
        if now - self._last_status_log >= 5.0:
            received, invalid, tracking_state, point_count = self._frame_queue.stats()
            if tracking_state is None:
                latest = "no valid frames received"
            else:
                latest = (
                    f"last_state={tracking_state} "
                    f"last_tracked_points={point_count}"
                )
            self.get_logger().info(
                f"TCP frames received={received}, invalid={invalid}, "
                f"ROS frames published={self._published}, "
                f"stereo image frames published={self._image_frames_published}; {latest}"
            )
            self._last_status_log = now


def main():
    parser = argparse.ArgumentParser(description="Publish Pi ORB-SLAM data to RViz2")
    parser.add_argument("--bind", default="0.0.0.0", help="TCP listen address")
    parser.add_argument("--port", type=int, default=8765, help="TCP listen port")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")

    frame_queue = LatestFrameQueue()
    server = FrameServer((args.bind, args.port), frame_queue)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)

    rclpy.init()
    node = RvizBridge(frame_queue)
    server_thread.start()
    node.get_logger().info(
        f"Listening on {args.bind}:{args.port}; publishing "
        "/orb_slam/pose, /orb_slam/path and /orb_slam/map_points"
        " plus /orb_slam/camera_left/image_raw and "
        "/orb_slam/camera_right/image_raw"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

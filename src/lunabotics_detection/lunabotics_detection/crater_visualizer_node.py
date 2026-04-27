#!/usr/bin/env python3
"""
Crater visualizer node.

Subscribes to /crater_edges (PointCloud2 of sampled rim points published by
crater_detector_node) and publishes a visualization_msgs/MarkerArray so that
RViz shows the fitted crater geometry — a ring, a centre sphere, and a text
label with the detected radius.

Optionally applies a temporal rolling-mean to smooth jittery detections.
Set smooth_window to 1 to disable smoothing.

Run alongside crater_detector_node:
  ros2 run lunabotics_detection crater_visualizer
"""

from collections import deque

import numpy as np
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _unpack_xyz(pc2_msg: PointCloud2) -> np.ndarray:
    """Extract XYZ columns from a PointCloud2 as an (N,3) float32 array."""
    n = pc2_msg.width * pc2_msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    step = pc2_msg.point_step
    data = bytes(pc2_msg.data)
    pts = np.frombuffer(data, dtype=np.float32).reshape(-1, step // 4)
    # x=offset 0, y=offset 4, z=offset 8 → columns 0,1,2
    return pts[:, :3].copy()


def _fit_circle_from_rim(pts: np.ndarray):
    """
    Given sampled rim points (N,3) in any frame, recover the circle parameters.
    Because crater_detector already sampled the circumference, the centroid IS
    the centre and the mean distance from centroid IS the radius.
    Returns (cx, cy, cz, radius) or None for empty input.
    """
    if len(pts) < 3:
        return None
    centre = pts.mean(axis=0)
    dists = np.linalg.norm(pts - centre, axis=1)
    radius = float(dists.mean())
    return float(centre[0]), float(centre[1]), float(centre[2]), radius


def _ring_marker(marker_id, frame_id, stamp, cx, cy, cz, radius,
                 r, g, b, lifetime_sec=0.5, n_pts=64):
    """LINE_STRIP marker tracing a horizontal circle."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'crater_ring'
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.03   # line width metres
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 0.9
    m.lifetime = Duration(sec=int(lifetime_sec),
                          nanosec=int((lifetime_sec % 1) * 1e9))
    from geometry_msgs.msg import Point
    angles = np.linspace(0, 2 * np.pi, n_pts + 1)  # +1 closes the loop
    for a in angles:
        p = Point()
        p.x = cx + radius * np.cos(a)
        p.y = cy + radius * np.sin(a)
        p.z = cz
        m.points.append(p)
    return m


def _sphere_marker(marker_id, frame_id, stamp, cx, cy, cz,
                   r, g, b, lifetime_sec=0.5):
    """SPHERE marker at the crater centre."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'crater_centre'
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = cx
    m.pose.position.y = cy
    m.pose.position.z = cz
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 0.12
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 1.0
    m.lifetime = Duration(sec=int(lifetime_sec),
                          nanosec=int((lifetime_sec % 1) * 1e9))
    return m


def _text_marker(marker_id, frame_id, stamp, cx, cy, cz, radius,
                 lifetime_sec=0.5):
    """TEXT_VIEW_FACING label showing radius above the crater centre."""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = 'crater_label'
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = cx
    m.pose.position.y = cy
    m.pose.position.z = cz + 0.3   # float label above the centre sphere
    m.pose.orientation.w = 1.0
    m.scale.z = 0.15               # text height metres
    m.color.r = m.color.g = m.color.b = 1.0
    m.color.a = 1.0
    m.text = f'r={radius:.2f}m'
    m.lifetime = Duration(sec=int(lifetime_sec),
                          nanosec=int((lifetime_sec % 1) * 1e9))
    return m


# Distinct colours for up to 8 simultaneous craters
_COLOURS = [
    (1.0, 0.2, 0.2),   # red
    (0.2, 0.8, 0.2),   # green
    (0.2, 0.5, 1.0),   # blue
    (1.0, 0.8, 0.0),   # yellow
    (1.0, 0.4, 0.0),   # orange
    (0.8, 0.2, 1.0),   # purple
    (0.0, 1.0, 0.9),   # cyan
    (1.0, 0.0, 0.6),   # pink
]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CraterVisualizerNode(Node):

    def __init__(self):
        super().__init__('crater_visualizer')

        self.declare_parameter('smooth_window', 3)   # frames to average; 1 = off
        self._win = max(1, self.get_parameter('smooth_window').value)

        # One deque per crater slot (keyed by index)
        self._history: dict[int, deque] = {}

        self.create_subscription(
            PointCloud2, '/crater_edges', self._edges_cb, 10
        )
        self._pub = self.create_publisher(MarkerArray, '/crater_markers', 10)

        self.get_logger().info(
            f'CraterVisualizer ready  smooth_window={self._win}'
        )

    def _edges_cb(self, msg: PointCloud2):
        pts = _unpack_xyz(msg)
        markers = MarkerArray()

        if len(pts) == 0:
            self._pub.publish(markers)
            return

        # The detector publishes 64 points per crater in sequence.
        # Split back into per-crater groups of 64.
        pts_per_crater = 64
        n_craters = len(pts) // pts_per_crater

        for i in range(n_craters):
            chunk = pts[i * pts_per_crater:(i + 1) * pts_per_crater]
            result = _fit_circle_from_rim(chunk)
            if result is None:
                continue
            cx, cy, cz, radius = result

            # Temporal smoothing via rolling mean
            if i not in self._history:
                self._history[i] = deque(maxlen=self._win)
            self._history[i].append((cx, cy, cz, radius))
            params = np.mean(self._history[i], axis=0)
            cx, cy, cz, radius = params

            self.get_logger().info(
                f'Crater {i}: centre=({cx:.3f}, {cy:.3f})  radius={radius:.3f}m',
                throttle_duration_sec=0.5,
            )

            r, g, b = _COLOURS[i % len(_COLOURS)]
            base_id = i * 3
            markers.markers.append(
                _ring_marker(base_id,     msg.header.frame_id, msg.header.stamp,
                             cx, cy, cz, radius, r, g, b)
            )
            markers.markers.append(
                _sphere_marker(base_id + 1, msg.header.frame_id, msg.header.stamp,
                               cx, cy, cz, r, g, b)
            )
            markers.markers.append(
                _text_marker(base_id + 2, msg.header.frame_id, msg.header.stamp,
                             cx, cy, cz, radius)
            )

        # Clear stale history slots that no longer have active craters
        for old_i in list(self._history):
            if old_i >= n_craters:
                del self._history[old_i]

        self._pub.publish(markers)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CraterVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

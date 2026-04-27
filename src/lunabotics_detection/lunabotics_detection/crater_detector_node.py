#!/usr/bin/env python3
"""
Crater rim detector — lunabotics competition.

Subscribes to the RealSense D435i depth image and detects crater rims by finding
significant depth discontinuities where the ground drops away. For each detected
crater, fits a circle to the rim points and publishes the circumference as a
PointCloud2 on /crater_edges (in the configured output_frame, default: odom).

Stability features:
  - Temporal depth averaging: rolls a window of raw depth frames before processing
    to suppress per-frame sensor noise.
  - RANSAC + algebraic LSQ refinement: RANSAC identifies inliers robustly;
    a deterministic algebraic least-squares fit on those inliers produces a stable
    centre/radius estimate that does not vary between runs on the same data.

Testing: visualise /crater_edges in RViz as a PointCloud2.
Production: add /crater_edges as an ObstacleLayer source in nav2 costmap params.
See TESTING.md for full instructions.
"""

import struct
from collections import deque

import cv2
import numpy as np

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from cv_bridge import CvBridge
import image_geometry
import tf2_ros


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _circle_from_3_points(p1, p2, p3):
    """
    Circumscribed circle through three 2-D points.
    Returns (cx, cy, radius) or None if the points are collinear.
    """
    ax, ay = p1
    bx, by = p2
    cx, cy = p3
    D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(D) < 1e-10:
        return None
    ux = (
        (ax**2 + ay**2) * (by - cy)
        + (bx**2 + by**2) * (cy - ay)
        + (cx**2 + cy**2) * (ay - by)
    ) / D
    uy = (
        (ax**2 + ay**2) * (cx - bx)
        + (bx**2 + by**2) * (ax - cx)
        + (cx**2 + cy**2) * (bx - ax)
    ) / D
    r = float(np.sqrt((ax - ux) ** 2 + (ay - uy) ** 2))
    return (ux, uy, r)


def _algebraic_circle_fit(pts_xy: np.ndarray):
    """
    Algebraic (linear) least-squares circle fit — deterministic, O(N) numpy.

    Solves the linear system  [x, y, 1] * [A, B, C]^T = -(x^2 + y^2)
    where  cx = -A/2,  cy = -B/2,  r = sqrt(cx^2 + cy^2 - C).

    Returns (cx, cy, r) or None if the system is degenerate.
    """
    x = pts_xy[:, 0]
    y = pts_xy[:, 1]
    A = np.stack([x, y, np.ones(len(x))], axis=1)
    b = -(x**2 + y**2)
    result, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 3:
        return None
    a_coef, b_coef, c_coef = result
    cx = -a_coef / 2.0
    cy = -b_coef / 2.0
    disc = cx**2 + cy**2 - c_coef
    if disc < 0:
        return None
    return cx, cy, float(np.sqrt(disc))


def _ransac_then_refine(points_xy: np.ndarray, n_iter: int = 80,
                        inlier_tol: float = 0.08):
    """
    Two-stage circle fit:
      1. RANSAC — robust against outliers, finds the best inlier set.
      2. Algebraic LSQ on inliers — deterministic, uses all inlier evidence,
         produces a stable centre/radius that does not vary between runs.

    Returns (cx, cy, r, n_inliers) or None.
    """
    pts = np.asarray(points_xy, dtype=np.float64)
    if len(pts) < 3:
        return None

    best_mask = None
    best_n = 0

    for _ in range(n_iter):
        idx = np.random.choice(len(pts), 3, replace=False)
        circle = _circle_from_3_points(pts[idx[0]], pts[idx[1]], pts[idx[2]])
        if circle is None:
            continue
        cx, cy, r = circle
        dists = np.abs(np.sqrt((pts[:, 0] - cx)**2 + (pts[:, 1] - cy)**2) - r)
        mask = dists < inlier_tol
        n = int(mask.sum())
        if n > best_n:
            best_n = n
            best_mask = mask

    if best_mask is None or best_n < 3:
        return None

    # Algebraic LSQ refinement on the inlier subset
    refined = _algebraic_circle_fit(pts[best_mask])
    if refined is None:
        return None
    cx, cy, r = refined
    return cx, cy, r, best_n


def _make_pointcloud2(points_xyz, frame_id, stamp):
    """Pack a list of (x, y, z) into a sensor_msgs/PointCloud2."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points_xyz)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 12  # 3 × float32
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    buf = bytearray(msg.row_step)
    for i, (x, y, z) in enumerate(points_xyz):
        struct.pack_into('fff', buf, i * 12, float(x), float(y), float(z))
    msg.data = bytes(buf)
    return msg


def _tf_to_Rt(tf_stamped):
    """Return (R 3x3, t 3-vec) from a geometry_msgs/TransformStamped."""
    tr = tf_stamped.transform
    tx, ty, tz = tr.translation.x, tr.translation.y, tr.translation.z
    qx, qy, qz, qw = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ], dtype=np.float64)
    t = np.array([tx, ty, tz], dtype=np.float64)
    return R, t


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CraterDetectorNode(Node):

    def __init__(self):
        super().__init__('crater_detector')

        # --- Parameters ---
        self.declare_parameter('edge_threshold',    0.05)   # metres
        self.declare_parameter('min_crater_radius', 0.15)   # metres
        self.declare_parameter('max_crater_radius', 0.4)    # metres
        self.declare_parameter('min_edge_points',   20)     # pixels
        self.declare_parameter('output_frame',      'camera_link')
        self.declare_parameter('processing_rate',   5.0)    # Hz
        self.declare_parameter('depth_avg_frames',  3)      # temporal averaging window

        self._edge_thr   = self.get_parameter('edge_threshold').value
        self._min_r      = self.get_parameter('min_crater_radius').value
        self._max_r      = self.get_parameter('max_crater_radius').value
        self._min_pts    = self.get_parameter('min_edge_points').value
        self._out_frame  = self.get_parameter('output_frame').value
        rate_hz          = self.get_parameter('processing_rate').value
        self._min_dt_sec = 1.0 / max(rate_hz, 0.1)
        avg_win          = max(1, self.get_parameter('depth_avg_frames').value)

        # Rolling buffer of raw uint16 depth frames for temporal averaging
        self._depth_buf: deque = deque(maxlen=avg_win)

        # --- Utilities ---
        self._bridge      = CvBridge()
        self._cam_model   = image_geometry.PinholeCameraModel()
        self._cam_ready   = False
        self._last_proc   = self.get_clock().now()
        self._frame_count = 0   # total frames received (for diagnostics)

        # --- TF ---
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- Subscriptions ---
        self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info',
            self._cam_info_cb, 1
        )
        self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self._depth_cb, 1
        )

        # --- Publisher ---
        self._pub = self.create_publisher(PointCloud2, '/crater_edges', 10)

        self.get_logger().info(
            f'CraterDetector ready — '
            f'edge_thr={self._edge_thr}m  '
            f'radius=[{self._min_r}, {self._max_r}]m  '
            f'output_frame={self._out_frame}  '
            f'rate={rate_hz}Hz  '
            f'depth_avg_frames={avg_win}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cam_info_cb(self, msg: CameraInfo):
        if not self._cam_ready:
            self._cam_model.fromCameraInfo(msg)
            self._fx = self._cam_model.fx()
            self._fy = self._cam_model.fy()
            self._cx = self._cam_model.cx()
            self._cy = self._cam_model.cy()
            self._cam_ready = True
            self.get_logger().info(
                f'Camera intrinsics received — '
                f'fx={self._fx:.1f}  fy={self._fy:.1f}  '
                f'cx={self._cx:.1f}  cy={self._cy:.1f}  '
                f'frame={self._cam_model.tfFrame()}'
            )

    def _depth_cb(self, msg: Image):
        if not self._cam_ready:
            return

        # Always accumulate into the temporal average buffer
        raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self._depth_buf.append(raw.astype(np.float32))
        self._frame_count += 1

        # Rate-limit the actual processing
        now = self.get_clock().now()
        if (now - self._last_proc).nanoseconds / 1e9 < self._min_dt_sec:
            return
        self._last_proc = now

        self._process(msg.header.stamp)

    def _process(self, stamp):
        # --- 1. Temporal depth average ---
        # Stack buffered frames; mark pixels invalid where ANY frame had depth=0
        stack = np.stack(self._depth_buf, axis=0)          # (W, H, C) → (N, H, W)
        any_invalid = np.any(stack == 0, axis=0)
        depth_avg = stack.mean(axis=0) / 1000.0            # uint16 mm → float32 m
        depth_avg[any_invalid] = 0.0

        n_valid = int((depth_avg > 0).sum())
        n_total = depth_avg.size
        self.get_logger().debug(
            f'[frame {self._frame_count}] depth avg over {len(self._depth_buf)} frames — '
            f'valid pixels: {n_valid}/{n_total} ({100*n_valid/n_total:.1f}%)'
        )

        # --- 2. Edge-preserving smooth ---
        depth_smooth = cv2.bilateralFilter(depth_avg, d=5, sigmaColor=0.15, sigmaSpace=5)
        depth_smooth[depth_avg == 0] = 0.0

        # --- 3. Rim detection ---
        # depth_step: how much deeper the local surroundings are vs the current pixel.
        # Large positive value = ground drops away here = crater rim candidate.
        dil_kernel = np.ones((7, 7), np.uint8)
        depth_local_max = cv2.dilate(depth_smooth, dil_kernel)
        depth_step = depth_local_max - depth_smooth

        sobel_x = cv2.Sobel(depth_smooth, cv2.CV_32F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(depth_smooth, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = np.sqrt(sobel_x**2 + sobel_y**2)

        valid_range = (depth_smooth > 0.3) & (depth_smooth < 6.0)
        rim_mask = (
            valid_range
            & (depth_step > self._edge_thr)
            & (grad_mag > self._edge_thr * 0.3)
        ).astype(np.uint8) * 255

        n_rim_pixels = int((rim_mask > 0).sum())
        self.get_logger().debug(
            f'[frame {self._frame_count}] rim mask — '
            f'{n_rim_pixels} candidate pixels  '
            f'(edge_thr={self._edge_thr}m  grad_thr={self._edge_thr*0.3:.3f}m/px)'
        )

        # --- 4. Morphological cleanup ---
        morph_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        rim_mask = cv2.morphologyEx(rim_mask, cv2.MORPH_CLOSE, morph_k)

        # --- 5. Connected components ---
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(rim_mask)
        n_components = n_labels - 1  # exclude background
        self.get_logger().debug(
            f'[frame {self._frame_count}] connected components: {n_components} '
            f'(min_pts filter={self._min_pts})'
        )

        if n_components == 0:
            self._pub.publish(_make_pointcloud2([], self._out_frame, stamp))
            return

        # --- 6. TF ---
        source_frame = self._cam_model.tfFrame()
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._out_frame, source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(
                f'TF {source_frame}→{self._out_frame} unavailable: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        R, t = _tf_to_Rt(tf_stamped)

        all_rim_pts = []
        n_accepted = 0

        for label in range(1, n_labels):
            area = stats[label, cv2.CC_STAT_AREA]
            if area < self._min_pts:
                self.get_logger().debug(
                    f'  component {label}: area={area}px — rejected (< min_edge_points={self._min_pts})'
                )
                continue

            ys, xs = np.where(labels == label)
            depths = depth_smooth[ys, xs]
            keep = depths > 0
            ys, xs, depths = ys[keep], xs[keep], depths[keep]
            if len(ys) < 3:
                continue

            depth_min = float(depths.min())
            depth_max = float(depths.max())
            depth_mean = float(depths.mean())

            # Subsample for RANSAC performance
            if len(xs) > 500:
                idx = np.random.choice(len(xs), 500, replace=False)
                xs, ys, depths = xs[idx], ys[idx], depths[idx]

            # --- 7. Vectorised deprojection ---
            d64 = depths.astype(np.float64)
            pts_cam = np.stack([
                (xs.astype(np.float64) - self._cx) * d64 / self._fx,
                (ys.astype(np.float64) - self._cy) * d64 / self._fy,
                d64,
            ], axis=1)

            # --- 8. Transform to output frame ---
            pts_world = (R @ pts_cam.T).T + t

            # --- 9. RANSAC + algebraic LSQ refinement ---
            result = _ransac_then_refine(pts_world[:, :2], n_iter=80, inlier_tol=0.08)

            if result is None:
                self.get_logger().debug(
                    f'  component {label}: area={area}px  depth=[{depth_min:.2f}, {depth_max:.2f}]m '
                    f'— RANSAC found no circle'
                )
                continue

            cx, cy, r, n_inliers = result
            inlier_ratio = n_inliers / len(pts_world) if len(pts_world) > 0 else 0.0

            if not (self._min_r <= r <= self._max_r):
                self.get_logger().debug(
                    f'  component {label}: area={area}px  depth_mean={depth_mean:.2f}m  '
                    f'r={r:.3f}m — rejected (outside [{self._min_r}, {self._max_r}]m)'
                )
                continue

            if n_inliers < self._min_pts:
                self.get_logger().debug(
                    f'  component {label}: area={area}px  r={r:.3f}m  '
                    f'inliers={n_inliers} — rejected (< min_edge_points={self._min_pts})'
                )
                continue

            self.get_logger().info(
                f'CRATER {n_accepted}: '
                f'centre=({cx:.3f}, {cy:.3f})m  '
                f'radius={r:.3f}m  '
                f'inliers={n_inliers} ({100*inlier_ratio:.0f}%)  '
                f'depth=[{depth_min:.2f}–{depth_max:.2f}]m mean={depth_mean:.2f}m  '
                f'rim_pixels={area}'
            )

            # --- 10. Sample rim circumference ---
            ground_z = float(np.mean(pts_world[:, 2]))
            angles = np.linspace(0.0, 2 * np.pi, 64, endpoint=False)
            for a in angles:
                all_rim_pts.append((cx + r * np.cos(a), cy + r * np.sin(a), ground_z))

            n_accepted += 1

        if n_accepted == 0:
            self.get_logger().debug(
                f'[frame {self._frame_count}] no craters passed all filters '
                f'(checked {n_components} components)'
            )

        self._pub.publish(
            _make_pointcloud2(all_rim_pts, self._out_frame, stamp)
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CraterDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

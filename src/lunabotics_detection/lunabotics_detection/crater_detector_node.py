"""
Crater detector using RealSense depth images.

Pipeline per frame (5 Hz):
  1. Temporal average over depth_avg_frames
  2. Bilateral filter
  3. Ground plane fit via least-squares on a centre-region sample
  4. Depression mask: depth below plane by > depth_drop_threshold
  5. Canny edge detection on depth image
  6. HoughCircles on dilated(depression_mask) AND Canny edges
  7. Validate: interior deeper than rim
  8. Unproject to 3D, transform to odom frame
  9. Publish PoseArray + MarkerArray
"""

import collections
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers geometry_msgs types with tf2


class CraterDetectorNode(Node):

    def __init__(self):
        super().__init__('crater_detector_node')

        self.declare_parameter('depth_drop_threshold', 0.15)
        self.declare_parameter('min_crater_radius_m', 0.10)
        self.declare_parameter('max_crater_radius_m', 0.25)
        self.declare_parameter('processing_rate', 5.0)
        self.declare_parameter('depth_avg_frames', 3)
        self.declare_parameter('output_frame', 'odom')

        self.drop_thresh   = self.get_parameter('depth_drop_threshold').value
        self.min_r_m       = self.get_parameter('min_crater_radius_m').value
        self.max_r_m       = self.get_parameter('max_crater_radius_m').value
        self.rate          = self.get_parameter('processing_rate').value
        self.avg_frames    = self.get_parameter('depth_avg_frames').value
        self.output_frame  = self.get_parameter('output_frame').value

        self.bridge    = CvBridge()
        self.cam_model = PinholeCameraModel()

        self.depth_deque: collections.deque = collections.deque(maxlen=self.avg_frames)
        self.latest_depth: np.ndarray | None = None
        self.cam_info_received = False

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.depth_sub    = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self._depth_cb, 10)
        self.info_sub     = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self._info_cb, 1)

        self.pose_pub   = self.create_publisher(PoseArray,   '/crater_detections', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/crater_markers',    10)

        self.timer = self.create_timer(1.0 / self.rate, self._process)

    def _info_cb(self, msg: CameraInfo):
        if not self.cam_info_received:
            self.cam_model.fromCameraInfo(msg)
            self.cam_info_received = True
            self.destroy_subscription(self.info_sub)

    def _depth_cb(self, msg: Image):
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_m = raw.astype(np.float32) * 0.001  # 16UC1 millimetres → metres
        depth_m[depth_m == 0] = np.nan
        self.depth_deque.append(depth_m)
        self.latest_depth_stamp = msg.header.stamp
        self.latest_depth_frame = msg.header.frame_id

    def _process(self):
        if not self.cam_info_received or len(self.depth_deque) < 1:
            return

        # 1. Temporal average
        stack = np.stack(list(self.depth_deque), axis=0)
        depth = np.nanmean(stack, axis=0)  # (H, W) float32 metres

        H, W = depth.shape
        fx = self.cam_model.fx()
        fy = self.cam_model.fy()
        cx = self.cam_model.cx()
        cy = self.cam_model.cy()

        # 2. Bilateral filter on valid depth (fill NaN with 0 for cv2)
        depth_filled = np.nan_to_num(depth, nan=0.0).astype(np.float32)
        depth_filt = cv2.bilateralFilter(depth_filled, d=9, sigmaColor=0.1, sigmaSpace=9)
        # Restore NaN mask
        depth_filt[depth_filled == 0] = np.nan

        # 3. Ground plane fit on centre 50% of image
        rh, rw = H // 4, W // 4
        roi_depth = depth_filt[rh: H - rh, rw: W - rw]
        roi_valid = np.isfinite(roi_depth) & (roi_depth > 0.1)

        if roi_valid.sum() < 100:
            return

        u_g, v_g = np.meshgrid(
            np.arange(rw, W - rw), np.arange(rh, H - rh))
        d_g = roi_depth

        mask = roi_valid
        u_s = u_g[mask].astype(np.float32)
        v_s = v_g[mask].astype(np.float32)
        d_s = d_g[mask]

        # Unproject to 3D rays, then fit plane ax+by+cz=1
        X = (u_s - cx) / fx * d_s
        Y = (v_s - cy) / fy * d_s
        Z = d_s
        A = np.column_stack([X, Y, Z])
        ones = np.ones(len(Z))
        plane, _, _, _ = np.linalg.lstsq(A, ones, rcond=None)  # [a,b,c]

        # Expected depth: plane · (ray * depth) = 1
        #   depth = 1 / (a*(u-cx)/fx + b*(v-cy)/fy + c)
        u_all, v_all = np.meshgrid(np.arange(W), np.arange(H))
        denom = (plane[0] * (u_all - cx) / fx +
                 plane[1] * (v_all - cy) / fy +
                 plane[2])
        with np.errstate(divide='ignore', invalid='ignore'):
            expected = np.where(np.abs(denom) > 1e-6, 1.0 / denom, np.nan).astype(np.float32)

        # 4. Depression mask: measured depth > expected + threshold
        valid = np.isfinite(depth_filt) & np.isfinite(expected) & (depth_filt > 0)
        depression = valid & (depth_filt > expected + self.drop_thresh)

        # 5. Canny on normalised depth
        d_norm = cv2.normalize(depth_filled, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        d_blur = cv2.GaussianBlur(d_norm, (5, 5), 0)
        edges  = cv2.Canny(d_blur, 30, 90)

        # 6. Rim candidates: dilated depression AND edges
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        dep_dilated = cv2.dilate(depression.astype(np.uint8) * 255, kernel)
        rim_candidates = cv2.bitwise_and(edges, dep_dilated)

        # Pixel radius bounds at median valid depth
        median_depth = float(np.nanmedian(depth_filt[valid])) if valid.any() else 1.0
        median_depth = max(median_depth, 0.3)
        min_r_px = max(3, int(self.min_r_m / median_depth * fx))
        max_r_px = max(min_r_px + 2, int(self.max_r_m / median_depth * fx))

        circles = cv2.HoughCircles(
            rim_candidates,
            cv2.HOUGH_GRADIENT,
            dp=1.5,
            minDist=max_r_px,
            param1=50,
            param2=15,
            minRadius=min_r_px,
            maxRadius=max_r_px,
        )

        detections_3d = []

        if circles is not None:
            circles = np.round(circles[0]).astype(int)
            for (u_c, v_c, r_px) in circles:
                # 7. Validate: interior mean depth > rim mean depth + threshold/2
                mask_inner = np.zeros((H, W), dtype=np.uint8)
                cv2.circle(mask_inner, (u_c, v_c), max(1, r_px // 2), 255, -1)
                mask_rim = np.zeros((H, W), dtype=np.uint8)
                cv2.circle(mask_rim, (u_c, v_c), r_px, 255, 2)

                inner_vals = depth_filt[mask_inner == 255]
                rim_vals   = depth_filt[mask_rim   == 255]
                inner_ok   = inner_vals[np.isfinite(inner_vals)]
                rim_ok     = rim_vals[np.isfinite(rim_vals)]

                if len(inner_ok) < 5 or len(rim_ok) < 3:
                    continue
                if inner_ok.mean() < rim_ok.mean() + self.drop_thresh / 2:
                    continue

                # 8. Unproject centre to 3D (camera frame)
                d_centre = float(depth_filt[v_c, u_c]) if np.isfinite(depth_filt[v_c, u_c]) else float(rim_ok.mean())
                if d_centre < 0.1:
                    continue

                X_c = (u_c - cx) / fx * d_centre
                Y_c = (v_c - cy) / fy * d_centre
                Z_c = d_centre

                # Physical radius from pixel radius and rim depth
                r_m = (r_px / fx) * float(rim_ok.mean())

                detections_3d.append((X_c, Y_c, Z_c, r_m))

        # 9. Transform to odom frame
        source_frame = getattr(self, 'latest_depth_frame', 'camera_depth_optical_frame')
        stamp        = getattr(self, 'latest_depth_stamp', self.get_clock().now().to_msg())

        try:
            tf = self.tf_buffer.lookup_transform(
                self.output_frame, source_frame,
                stamp, timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        pose_array = PoseArray()
        pose_array.header.stamp    = stamp
        pose_array.header.frame_id = self.output_frame

        marker_array = MarkerArray()
        now_msg      = self.get_clock().now().to_msg()

        for i, (X_c, Y_c, Z_c, r_m) in enumerate(detections_3d):
            pt_cam = Point(x=X_c, y=Y_c, z=Z_c)
            pt_world = _transform_point(pt_cam, tf)

            p = Pose()
            p.position    = pt_world
            p.orientation.w = 1.0
            pose_array.poses.append(p)

            # Ring marker at crater rim height
            ring = Marker()
            ring.header.stamp    = now_msg
            ring.header.frame_id = self.output_frame
            ring.ns              = 'craters'
            ring.id              = i * 2
            ring.type            = Marker.CYLINDER
            ring.action          = Marker.ADD
            ring.pose.position   = pt_world
            ring.pose.orientation.w = 1.0
            ring.scale.x = r_m * 2
            ring.scale.y = r_m * 2
            ring.scale.z = 0.05
            ring.color   = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.6)
            ring.lifetime.sec    = 2
            marker_array.markers.append(ring)

        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)


def _transform_point(pt: Point, tf_stamped) -> Point:
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation

    # Rotate point by quaternion (q * p * q^-1)
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    px, py, pz = pt.x, pt.y, pt.z

    # Quaternion rotation: efficient formula
    tx = 2.0 * (qy * pz - qz * py)
    ty = 2.0 * (qz * px - qx * pz)
    tz = 2.0 * (qx * py - qy * px)

    rx = px + qw * tx + qy * tz - qz * ty
    ry = py + qw * ty + qz * tx - qx * tz
    rz = pz + qw * tz + qx * ty - qy * tx

    return Point(x=rx + t.x, y=ry + t.y, z=rz + t.z)


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

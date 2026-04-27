# lunabotics_detection

Python ROS 2 package for **crater detection** from RealSense D435i depth images.
Publishes detected craters in the `odom` frame so Nav2 can mark them in the costmap.

---

## Nodes

### `crater_detector_node`

Processes depth images at 5 Hz and detects depressions (craters).

**Subscriptions:**
- `/camera/camera/depth/image_rect_raw` (`sensor_msgs/Image`, 16UC1 mm)
- `/camera/camera/depth/camera_info` (`sensor_msgs/CameraInfo`)

**Publications:**
- `/crater_detections` (`geometry_msgs/PoseArray`, frame = `odom`) — one pose per crater centre
- `/crater_markers` (`visualization_msgs/MarkerArray`) — cylinder markers for RViz

### `crater_cloud_publisher_node`

Converts `/crater_detections` PoseArray → `/crater_cloud` PointCloud2 so the Nav2
obstacle layer can subscribe to it.

**Subscriptions:** `/crater_detections`  
**Publications:** `/crater_cloud` (`sensor_msgs/PointCloud2`, frame = `odom`)

---

## Detection Pipeline

```
depth image (16UC1, mm)
    │
    ▼ temporal average (3 frames)
    ▼ bilateral filter
    ▼ ground plane fit (least-squares on centre 50% of image)
    ▼ depression mask: measured depth > plane depth + threshold
    ▼ Canny edges on depth image
    ▼ rim candidates = dilate(depression_mask) AND Canny
    ▼ HoughCircles on rim candidates
    ▼ validate: interior mean depth > rim mean depth + threshold/2
    ▼ unproject circle centre to 3D (camera frame)
    ▼ TF: camera_depth_optical_frame → odom
    ▼
/crater_detections  /crater_markers
```

Crater size per rulebook: **40–50 cm diameter** → detection range `[0.10 m, 0.25 m]` radius.

---

## Parameters (`config/crater_detection.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `depth_drop_threshold` | 0.15 m | Depth below ground plane to count as crater |
| `min_crater_radius_m` | 0.10 m | ~20 cm min diameter |
| `max_crater_radius_m` | 0.25 m | ~50 cm max diameter (rulebook limit) |
| `processing_rate` | 5.0 Hz | Timer rate |
| `depth_avg_frames` | 3 | Temporal average window |
| `output_frame` | `odom` | TF target frame for outputs |

---

## Running Standalone

```bash
source install/setup.bash
ros2 launch lunabotics_detection crater_detection.launch.py
```

---

## Verification

```bash
# Point RealSense at a 20–50 cm depression on the floor

# Watch detections (should see PoseArray with poses near the crater)
ros2 topic echo /crater_detections

# Watch the cloud (should have ~1 point per crater)
ros2 topic echo /crater_cloud --once

# In RViz: add MarkerArray display on /crater_markers
# Expect cyan cylinder rings floating above detected craters
```

**If no detections appear:**
- Check that the robot TF chain is live (`odom → base_link → camera_depth_optical_frame`)
- Lower `depth_drop_threshold` to 0.08 m for shallow craters
- Ensure the camera is not too far away — HoughCircles pixel radius is computed from
  `crater_radius_m / depth * fx`, so at >2 m range the feature shrinks below `minRadius`

---

## Nav2 Integration

`/crater_cloud` feeds the `crater_layer` (ObstacleLayer) in both the global and local costmaps
configured in `lunabotics_bringup/config/nav2_params.yaml`. Points persist for 10 s
(`observation_persistence: 10.0`) so a crater remains in the costmap after the robot drives
past and can no longer see it.

---

## Files

| File | Description |
|------|-------------|
| `lunabotics_detection/crater_detector_node.py` | Main detection pipeline |
| `lunabotics_detection/crater_cloud_publisher_node.py` | PoseArray → PointCloud2 bridge |
| `config/crater_detection.yaml` | ROS 2 params |
| `launch/crater_detection.launch.py` | Launches both nodes |

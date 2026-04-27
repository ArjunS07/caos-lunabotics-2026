# Testing: `lunabotics_detection` crater rim detector

## Prerequisites

Install ROS 2 Humble dependencies (one-time, on the Jetson):

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-image-geometry \
  ros-humble-tf2-sensor-msgs
```

Python deps (`numpy` and `opencv-python` should already be present):

```bash
pip3 install numpy opencv-python
```

---

## Build

```bash
cd ~/lunabotics-development/lunabotics_ws
colcon build --packages-select lunabotics_detection
source install/setup.bash
```

Use `--symlink-install` to skip rebuilds while editing Python files:

```bash
colcon build --symlink-install --packages-select lunabotics_detection
source install/setup.bash
```

---

## Running

The crater detector subscribes to the RealSense depth stream, so the camera
must be running first.

**Terminal 1 — sensors (RealSense + TF):**

```bash
ros2 launch lunabotics_bringup launch_jetson.py
```

**Terminal 2 — crater detector:**

```bash
ros2 launch lunabotics_detection crater_detection.launch.py
```

---

## Verifying output

Check that the topic is publishing:

```bash
ros2 topic hz /crater_edges     # should tick at ~5 Hz
ros2 topic info /crater_edges   # confirm type: sensor_msgs/PointCloud2
```

Inspect a single message (will be empty when no craters detected):

```bash
ros2 topic echo /crater_edges --once
```

Watch the node log for detected craters:

```bash
ros2 node info /crater_detector
# or just watch Terminal 2 — each detected crater prints:
# [INFO] Crater: centre=(x.xx, y.yy)m  radius=r.rrm  inliers=N
```

---

## Output frame: testing vs. production

`output_frame` in `crater_detection.yaml` controls which TF frame `/crater_edges`
is published in. It must be reachable from `camera_depth_optical_frame`.

| Mode | `output_frame` | Requires |
|---|---|---|
| Testing (default) | `camera_link` | RealSense running |
| Production | `odom` | Full stack: rf2o/EKF publishing `odom→base_link` |

`camera_link` is always available once the RealSense driver starts — no
localization needed. The XY ground plane is ~15° off true level (camera tilt)
but crater shape is still valid for visual validation.

Switch to `odom` for Nav2 costmap integration so crater circles are properly
projected onto the flat ground plane.

---

## RViz visualisation

1. Open RViz (`ros2 run rviz2 rviz2` or via `launch_local.py`)
2. Add → **PointCloud2**
3. Set **Topic** to `/crater_edges`
4. Set **Fixed Frame** to `odom`
5. Set **Size** to `0.05` and **Style** to `Spheres` for visibility

Point the camera at a bucket, cardboard box edge, or any sharp drop-off to
simulate a crater rim. A ring of points should appear at the fitted circle
circumference.

---

## Tuning parameters

Edit `config/crater_detection.yaml` and rebuild (or use `--symlink-install`).

| Parameter | Symptom if too low | Symptom if too high |
|---|---|---|
| `edge_threshold` | False positives on floor texture | Shallow craters missed |
| `min_crater_radius` | Noise rings appear near camera | Small craters ignored |
| `max_crater_radius` | — | Large craters ignored |
| `min_edge_points` | Noise clusters attempt fits | Partial rim arcs rejected |
| `processing_rate` | High CPU | Sluggish detection |

---

## Nav2 costmap integration (production)

Once detection is validated, add `/crater_edges` as an additional obstacle
source in the Nav2 local/global costmap configuration. No code changes needed —
only the YAML:

```yaml
# In your nav2_params.yaml, under local_costmap or global_costmap:
local_costmap:
  ros__parameters:
    plugins: ["obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: scan crater_edges
      scan:
        topic: /scan
        data_type: LaserScan
        marking: true
        clearing: true
      crater_edges:
        topic: /crater_edges
        data_type: PointCloud2
        marking: true
        clearing: false         # keep crater marked even after robot moves past rim
        obstacle_max_range: 5.0
        min_obstacle_height: -0.1
        max_obstacle_height: 0.5
```

This adds the crater rim circles to the obstacle layer, causing the inflation
layer to grow a keep-out zone around each detected crater automatically.

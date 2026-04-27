# Debugging & Initial Setup Guide

Step-by-step component verification for the Lunabotics autonomy stack.
Test each stage before moving to the next. Each component has its own launch file so
they can be validated independently — or by different people simultaneously (see bottom).

---

## 0. Build & Network Setup

Run once per session on the Jetson and on every laptop that needs to see topics.

```bash
# On the Jetson
cd ~/lunabotics-development/lunabotics_ws
colcon build --symlink-install
source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

```bash
# On each laptop
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
# Source the workspace too if you want to run ros2 launch from the laptop:
# source ~/lunabotics-development/lunabotics_ws/install/setup.bash
rviz2
```

Laptops and the Jetson must be on the same LAN for DDS discovery to work.

---

## 1. LiDAR + TF

**What this tests:** Unitree L2 driver, `base_link → unilidar_lidar` static TF, raw point cloud.

```bash
# Jetson — launch TF and LiDAR driver
ros2 launch lunabotics_bringup tf_base_link.launch.py &
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node \
  --ros-args \
  -p cloud_frame:=unilidar_lidar \
  -p cloud_topic:=unilidar/cloud \
  -p serial_port:=/dev/ttyACM0 \
  -p baudrate:=4000000
```

**Check (Jetson terminal):**
```bash
ros2 topic hz /unilidar/cloud          # expect ~10 Hz
ros2 topic echo /unilidar/imu --once   # should have angular_velocity + linear_acceleration
ros2 run tf2_ros tf2_echo base_link unilidar_lidar   # should print static transform
```

**RViz (laptop):**
- Add `PointCloud2` → topic `/unilidar/cloud`
- Fixed Frame: `base_link`
- You should see a live spinning point cloud of the room

---

## 2. RealSense D435i

**What this tests:** Depth camera driver, camera info, depth image stream.

```bash
# Jetson
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  depth_module.profile:=640x480x30 \
  ordered_pc:=true
```

**Check (Jetson terminal):**
```bash
ros2 topic hz /camera/camera/depth/image_rect_raw   # expect ~30 Hz
ros2 topic echo /camera/camera/depth/camera_info --once
```

**RViz (laptop):**
- Add `Image` → topic `/camera/camera/depth/image_rect_raw`
- Dark pixels = close, bright = far

---

## 3. ICP Localization

**What this tests:** GICP scan-to-submap odometry, IMU integration, odom→base_link TF.
**Requires:** LiDAR running (step 1).

```bash
# Jetson
ros2 launch lunabotics_icp_localization icp_localization.launch.py
```

**Check (Jetson terminal):**
```bash
ros2 topic echo /odometry/filtered --once
# Expect: position near (0,0,0), orientation near identity (w≈1)

ros2 run tf2_ros tf2_echo odom base_link
# Should print a live transform updating at ~10 Hz
```

**Stress test:** Push the robot ~1–2 m by hand — `/odometry/filtered` position should track it.
Rotate 90° — yaw should change by ~1.57 rad.

**RViz (laptop):**
- Fixed Frame: `odom`
- Add `Odometry` → topic `/odometry/filtered`
- The arrow should move as the robot moves

**Watch for warnings:**
```bash
ros2 topic echo /rosout | grep icp
# "GICP did not converge" once on startup is fine (submap seeding)
# Repeated warnings = too few features in view; check the range clip param
```

**Competition compliance check:** Drive the robot near a wall.
The "GICP did not converge" warning should *not* appear — walls are excluded by the 3 m range clip.
If convergence suddenly improves near walls, lower `icp_range_clip` to 2.5 m in
`src/lunabotics_icp_localization/config/icp_localization.yaml`.

---

## 4. Crater Detection

**What this tests:** Ground-plane fit, HoughCircles, TF transform to odom frame.
**Requires:** RealSense (step 2) + ICP running for TF (step 3).

```bash
# Jetson
ros2 launch lunabotics_detection crater_detection.launch.py
```

**Check (Jetson terminal):**
```bash
ros2 topic echo /crater_detections
# Point camera at a depression (box lid, bucket, ~20–50 cm wide)
# Expect: PoseArray with one pose near the depression in odom frame

ros2 topic echo /crater_cloud --once
# Should have 1 point per detected crater
```

**RViz (laptop):**
- Add `MarkerArray` → topic `/crater_markers`
- Aim camera at a ~30 cm depression — cyan cylinder ring should appear above it

**If nothing detects:**
```bash
# Temporarily lower the depth threshold (live param change, no restart needed)
ros2 param set /crater_detector_node depth_drop_threshold 0.08
```

---

## 5. Nav2 Stack

**What this tests:** Global planner (A*), local controller (RPP), global and local costmaps.
**Requires:** ICP running (`odom → base_link` TF must be live).

```bash
# Jetson
ros2 launch lunabotics_bringup navigation.launch.py
```

**Check (Jetson terminal):**
```bash
# Lifecycle nodes should all be active
ros2 node list | grep -E "planner|controller|bt_navigator|costmap"

# Costmaps publishing
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

**RViz (laptop):**
- Fixed Frame: `odom`
- Add `Map` → topic `/global_costmap/costmap` — should see an empty 8×6 m grid
- Place a large object in front of the LiDAR — it should appear as inflated red/yellow cells within ~1 s
- Use the **Nav2 Goal** tool (the green arrow button) to click a point ~2 m ahead of the robot
  — the robot should plan a path and drive to it

---

## 6. Full Stack + Mission FSM

**What this tests:** End-to-end autonomy from trigger to arrival.

```bash
# Jetson — everything
ros2 launch lunabotics_bringup launch_jetson.py
```

**Check state machine:**
```bash
ros2 topic echo /mission_state
# Should print "IDLE" once per second
```

**Trigger autonomy:**
```bash
ros2 service call /autonomy_start std_srvs/srv/Trigger
# Response: success: True, message: "Autonomy started..."
# /mission_state should switch to "TRAVERSING"
```

**Watch progress:**
```bash
# x should increase as robot drives toward goal at (5.5 m, 1.25 m)
ros2 topic echo /odometry/filtered | grep -A3 position
```

**Emergency stop:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{}' --once
```

---

## Quick Sanity One-Liners

```bash
# See all relevant topics at once
ros2 topic list | grep -E "cloud|odom|crater|mission|costmap|cmd_vel"

# Generate TF tree PDF
ros2 run tf2_tools view_frames && evince frames.pdf

# Check a topic is alive and its rate
ros2 topic hz /odometry/filtered

# Watch Nav2 feedback live
ros2 topic echo /navigate_to_pose/_action/feedback
```

---

## Splitting Testing Across People

Every component has its own launch file and can be run independently — by separate SSH
sessions into the Jetson, or by people on laptops on the same LAN.

### How the roles split in practice

**Rule of thumb:**
- SSH sessions into the Jetson → run nodes, echo topics, trigger services
- Laptops on the same LAN → run RViz (don't use `ssh -X` for RViz; point cloud bandwidth makes it unusable)
- `tmux` on the Jetson → run multiple components in parallel without needing separate SSH connections

```
┌─────────────────────────────────────┐      ┌──────────────────────┐
│  Jetson (SSH sessions or tmux)      │      │  Laptop(s) on LAN    │
│                                     │ DDS  │                      │
│  session 1: sensors (LiDAR + RS)    │◄────►│  RViz                │
│  session 2: ICP localization        │      │  ros2 topic echo     │
│  session 3: crater detection        │      │  ros2 service call   │
│  session 4: Nav2                    │      │                      │
│  session 5: mission FSM             │      └──────────────────────┘
└─────────────────────────────────────┘
```

**Recommended tmux setup on the Jetson:**

```bash
# SSH in, then start tmux
ssh caos@<jetson-ip>
tmux new-session -s lunabotics

# Open panes: Ctrl-B then " (horizontal split) or % (vertical split)
# Switch panes: Ctrl-B then arrow keys
# Detach and leave running: Ctrl-B then D
# Reattach later: tmux attach -t lunabotics
```

---

### Assignment table

| Person | Where they work | What they run | What to watch |
|--------|----------------|---------------|---------------|
| **A** | Jetson SSH session 1 | LiDAR + TF (step 1) | `ros2 topic hz /unilidar/cloud` |
| **B** | Jetson SSH session 2 | RealSense (step 2) | `ros2 topic hz /camera/camera/depth/image_rect_raw` |
| **C** | Jetson SSH session 3 | ICP localization (step 3) | `ros2 topic echo /odometry/filtered` |
| **D** | Jetson SSH session 4 | Crater detection (step 4) | `ros2 topic echo /crater_detections` |
| **E** | Jetson SSH session 5 | Nav2 (step 5) | `ros2 topic hz /global_costmap/costmap` |
| **F** | Laptop | RViz — visualise everything | Point cloud, odometry, costmap, crater markers |
| **G** | Any laptop or SSH | Mission FSM trigger + monitoring | `ros2 topic echo /mission_state` |

A, B, C, D, E can all work in parallel on the Jetson — they just need steps to be started
in order (sensors before ICP, ICP before Nav2/detection).

---

### Option: Testing without the Jetson (bag files)

Record sensor data once on the Jetson, then anyone can replay it on their own laptop
with no hardware needed:

```bash
# Record on Jetson — run this while driving the robot around
ros2 bag record \
  /unilidar/cloud /unilidar/imu \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /tf /tf_static \
  -o lunabotics_sensors
```

```bash
# Replay on any machine (laptop, desktop, CI)
ros2 bag play lunabotics_sensors --clock

# Then launch just the component you want to test, e.g. ICP only:
ros2 launch lunabotics_icp_localization icp_localization.launch.py

# Or crater detection only:
ros2 launch lunabotics_detection crater_detection.launch.py
```

This lets people debug ICP drift, tune crater thresholds, or test Nav2 costmap behaviour
entirely offline — no robot, no Jetson needed.

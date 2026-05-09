# lunabotics_bringup

Top-level launch and configuration package. Contains no executable nodes — only launch files and
YAML configs that wire the rest of the stack together.

---

## Directory Layout

```
lunabotics_bringup/
├── config/
│   ├── nav2_params.yaml        # Full Nav2 stack config (planner, controller, costmaps)
│   └── elevation_mapping.yaml  # ANYbotics elevation_mapping config (optional terrain layer)
├── launch/
│   ├── robot.launch.py         # PRIMARY (Jetson): hardware + processing, no Mission FSM or RViz
│   ├── laptop.launch.py        # PRIMARY (laptop): Mission FSM + RViz only
│   ├── launch_jetson.py        # Legacy all-in-one: full stack on one machine, no RViz
│   ├── launch.py               # Dev: same as launch_jetson.py + RViz
│   ├── navigation.launch.py    # Includes nav2_bringup with nav2_params.yaml
│   └── tf_base_link.launch.py  # Static TF: base_link → unilidar_lidar
└── rviz/
    └── view.rviz               # RViz config
```

---

## Launch Files

### Competition: split Jetson + laptop

The preferred competition configuration splits the stack across two machines:

**Jetson** — hardware, processing, and navigation:
```bash
export ROS_DOMAIN_ID=42 && export ROS_LOCALHOST_ONLY=0
ros2 launch lunabotics_bringup robot.launch.py
```

**Operator laptop** — Mission FSM and RViz:
```bash
export ROS_DOMAIN_ID=42 && export ROS_LOCALHOST_ONLY=0
ros2 launch lunabotics_bringup laptop.launch.py
```

`robot.launch.py` starts (in order):

| Component | Package / Node |
|-----------|---------------|
| RealSense D435i | `realsense2_camera` |
| Unitree L2 LiDAR | `unitree_lidar_ros2` |
| base_link → unilidar_lidar TF | `tf_base_link.launch.py` |
| unilidar_lidar → camera_link TF | static_transform_publisher |
| ICP localization | `lunabotics_icp_localization` |
| Crater detection | `lunabotics_detection` |
| Nav2 (planner + controller + costmaps) | `navigation.launch.py` |

`laptop.launch.py` starts:

| Component | Package / Node |
|-----------|---------------|
| Mission FSM | `lunabotics_mission` |
| RViz | `rviz2` |

### `launch_jetson.py` — Legacy all-in-one

Runs the full stack (including Mission FSM) on a single machine without RViz. Kept for bench
testing when a second laptop is not available.

### `launch.py` — Development (single machine with RViz)

Same as `launch_jetson.py` plus `rviz2`.

### `navigation.launch.py`

Thin wrapper that includes `nav2_bringup/navigation_launch.py` pointing at
`config/nav2_params.yaml`. Import this instead of calling nav2_bringup directly so the config
path resolves correctly from the installed share directory.

---

## Key Configuration Files

### `config/nav2_params.yaml`

Configures the full Nav2 stack. Key tuning values:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `desired_linear_vel` | 0.25 m/s | Conservative for regolith |
| `inflation_radius` | 1.2 m | Wide clearance — slalom path planning |
| `robot_radius` | 0.40 m | Update to actual footprint |
| `xy_goal_tolerance` | 0.50 m | 50 cm arrival window |
| `allow_unknown` | true | No prior map — costmap starts empty |
| `transform_tolerance` | 1.0 s | All Nav2 servers (costmaps, controller, behavior); covers GICP latency on Jetson |
| global costmap `origin_x/y` | -0.5, -3.5 m | Robot starts 0.5 m inside left edge, centred laterally |
| global costmap `width × height` | 10 × 7 m | Covers 6.88 m arena with ~1.5 m margins |
| `default_nav_to_pose_bt_xml` | `navigate_to_pose_w_replanning_and_recovery.xml` | Humble-compatible BT; `RemovePassedGoals` not available in Humble |
| `default_nav_through_poses_bt_xml` | same | Avoids `navigate_through_poses_w_replanning_and_recovery.xml` which crashes bt_navigator |

Obstacle layers in both global and local costmaps:
- `crater_layer` — subscribes to `/crater_cloud` (PointCloud2) from `lunabotics_detection`
- `obstacle_layer` — subscribes to `/unilidar/cloud` (full-range LiDAR)
- `inflation_layer` — 1.2 m inflation radius around all obstacles

### `config/elevation_mapping.yaml`

Configures ANYbotics `elevation_mapping`. Subscribes to `/unilidar/cloud` (full range, **not**
clipped) and `/odometry/filtered`. Produces a `GridMap` on `/elevation_map` covering 8 m × 6 m
at 5 cm resolution. Used as an optional terrain layer — high-variance cells indicate rough terrain.

To add the elevation layer back to the Nav2 costmap, uncomment the `elevation_layer` block in
`nav2_params.yaml` once `grid_map_costmap_2d` is confirmed installed.

---

## TF Tree

```
odom
 └── base_link          ← published by lunabotics_icp_localization at LiDAR rate
      └── unilidar_lidar  ← static (tf_base_link.launch.py: 0.20 m forward, 0.60 m up)
           └── camera_link  ← static (co-located, -38° pitch to face ground)
                └── camera_depth_optical_frame  ← published by realsense2_camera
```

All Nav2 servers set `transform_tolerance: 1.0` to cover GICP latency on the Jetson.  ICP
publishes an IMU-predicted TF immediately at the scan timestamp so Nav2's message filter
doesn't wait for GICP; the corrected TF follows at `stamp+1 ns` once GICP converges.

---

## Network Setup (Jetson ↔ Laptop)

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Set on **both** the Jetson and any laptop that needs to see topics.

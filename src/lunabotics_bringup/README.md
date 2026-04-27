# lunabotics_bringup

Top-level launch and configuration package. Contains no executable nodes — only launch files and
YAML configs that wire the rest of the stack together.

---

## Directory Layout

```
lunabotics_bringup/
├── config/
│   ├── nav2_params.yaml        # Full Nav2 stack config (planner, controller, costmaps)
│   └── elevation_mapping.yaml  # ANYbotics elevation_mapping config
├── launch/
│   ├── launch_jetson.py        # PRIMARY: Jetson competition launch (no RViz)
│   ├── launch.py               # Dev: same as jetson + RViz
│   ├── launch_local.py         # RViz-only, for a remote laptop
│   ├── navigation.launch.py    # Includes nav2_bringup with nav2_params.yaml
│   ├── tf_base_link.launch.py  # Static TF: base_link → unilidar_lidar
│   └── launch_bench_mvp.launch.py  # LiDAR-only bench test
└── rviz/
    └── view.rviz               # RViz config
```

---

## Launch Files

### `launch_jetson.py` — Competition launch

Starts the full autonomy stack on the Jetson. Does **not** start RViz.

```bash
source install/setup.bash
ros2 launch lunabotics_bringup launch_jetson.py
```

Nodes and includes, in startup order:

| Component | Package / Node |
|-----------|---------------|
| RealSense D435i | `realsense2_camera` |
| Unitree L2 LiDAR | `unitree_lidar_ros2` |
| base_link → unilidar_lidar TF | `tf_base_link.launch.py` |
| unilidar_lidar → camera_link TF | static_transform_publisher |
| ICP localization | `lunabotics_icp_localization` |
| Elevation mapping | `elevation_mapping` |
| Crater detection | `lunabotics_detection` |
| Nav2 (planner + controller + costmaps) | `navigation.launch.py` |
| Mission FSM | `lunabotics_mission` |

### `launch.py` — Development (single machine)

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
      └── unilidar_lidar  ← static (tf_base_link.launch.py)
           └── camera_link  ← static (30 cm forward, -15° pitch)
                └── camera_depth_optical_frame  ← published by realsense2_camera
```

---

## Network Setup (Jetson ↔ Laptop)

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Set on **both** the Jetson and any laptop that needs to see topics.

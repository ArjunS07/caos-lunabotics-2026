# caos-lunabotics-2026

ROS 2 **Humble** workspace for NASA Lunabotics 2026 — **Travel Automation (250 pts)**.

The robot autonomously traverses from the Starting/Excavation Zone, through the Obstacle Zone
(boulders + craters + central column), and stops inside the Construction Zone.  
We do **not** implement excavation or berm deposit — only the traverse.

**Hardware:** NVIDIA Jetson Orin Nano · Unitree L2 LiDAR · Intel RealSense D435i

---

## Package Overview

| Package | Language | Role |
|---------|----------|------|
| [`lunabotics_bringup`](src/lunabotics_bringup/README.md) | Python (launch) | Top-level launch files, Nav2 and sensor configs |
| [`lunabotics_icp_localization`](src/lunabotics_icp_localization/README.md) | C++ | GICP scan-to-submap odometry; publishes `odom→base_link` TF |
| [`lunabotics_detection`](src/lunabotics_detection/README.md) | Python | Crater detection from RealSense depth; feeds Nav2 costmap |
| [`lunabotics_mission`](src/lunabotics_mission/README.md) | Python | Competition FSM — waits for trigger, sends Nav2 goal, announces arrival |

External packages pulled via `ros2.repos`:
- `rf2o_laser_odometry` — kept in repos but no longer launched (replaced by ICP)
- `elevation_mapping` (ANYbotics) — 2.5D terrain map from LiDAR
- `grid_map` (ANYbotics) — GridMap library used by elevation_mapping

---

## First-Time Setup

```bash
git clone https://github.com/ArjunS07/caos-lunabotics-2026.git lunabotics_ws
cd lunabotics_ws

# Pull external packages
vcs import src < ros2.repos

# Install system dependencies
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

---

## Running (competition: Jetson + laptop)

**On the Jetson** — sensors, localization, detection, Nav2:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source install/setup.bash
ros2 launch lunabotics_bringup robot.launch.py
```

**On the operator laptop** — Mission FSM and RViz:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
source install/setup.bash
ros2 launch lunabotics_bringup laptop.launch.py
```

**Pre-launch checklist** (operator, before hands-free):
1. Disable magnetometer on Unitree L2 IMU (see [ICP README](src/lunabotics_icp_localization/README.md#competition-compliance))
2. Place robot in Excavation Zone, rotate it to face the Construction Zone using remote control
3. Confirm `/odometry/filtered` is publishing and the odom→base_link TF looks sane in RViz
4. Announce to MCJ you are going hands-free
5. Trigger autonomy:

```bash
ros2 service call /autonomy_start std_srvs/srv/Trigger
```

6. Announce to MCJ when autonomy begins (immediately after the call returns success)
7. Announce when `/mission_state` publishes `ARRIVED`

---

## Running on a Development Machine (single PC with RViz)

```bash
source install/setup.bash
ros2 launch lunabotics_bringup launch.py
```

All nodes and RViz on one machine — use when a Jetson is not available.

---

## Monitoring During a Run

```bash
# Watch mission state (IDLE → TRAVERSING → ARRIVED)
ros2 topic echo /mission_state

# Watch odometry
ros2 topic echo /odometry/filtered

# Watch crater detections
ros2 topic echo /crater_detections

# TF tree
ros2 run tf2_tools view_frames
```

---

## Arena Geometry Reference

| Zone | Distance from start wall | Notes |
|------|--------------------------|-------|
| Starting Zone | 0–2 m | Robot placed here; no obstacles |
| Excavation Zone | 0–2.5 m | Robot declares hands-free here |
| Obstacle Zone | 2.5–6.88 m | Boulders, craters, central column |
| Construction Zone | ~last 1.5 m (offset) | Nav2 goal target: (5.5 m, 1.25 m) |

Arena total: **6.88 m × 5.0 m**. Starting pose in odom frame: **(0, 0, 0°)** after operator alignment.

---

## Other Launch Files

| File | Purpose |
|------|---------|
| `robot.launch.py` | Jetson: hardware + processing (no Mission FSM, no RViz) |
| `laptop.launch.py` | Operator laptop: Mission FSM + RViz only |
| `launch_jetson.py` | Legacy all-in-one: full stack on one machine, no RViz |
| `launch.py` | Dev: full stack + RViz on one machine |

---

## Hardware Notes

See [`Description.md`](Description.md) for sensor layout.

## License

Apache-2.0 (see [LICENSE](LICENSE)).


### Directory structure

- This directory contains a symlink to `home/caos/lunabotics-development/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2` in `unitree_lidar_ros2`

### Launching
Run:
```bash
cd ~/lunabotics-development/lunabotics_ws
colcon build          # first time only, or after code changes
source install/setup.bash
ros2 launch lunabotics_bringup launch.py
```

# caos-lunabotics-2026

ROS 2 **Humble** workspace for NASA Lunabotics: Unitree L2 LiDAR, Intel RealSense D435i, `pointcloud_to_laserscan`, `octomap_server`, lidar odometry (**rf2o**), optional **robot_localization** EKF + depth→`LaserScan` (`/scan_depth`).

## Prerequisites

- Ubuntu 22.04 + [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- External packages: `unitree_lidar_ros2`, `realsense2_camera` (for full desktop launch), `pointcloud_to_laserscan`, `octomap_server`, `robot_localization`, `depthimage_to_laserscan`
- **rf2o** is pulled in via `ros2.repos` (not on apt for most setups)

## Clone and build

```bash
git clone https://github.com/ArjunS07/caos-lunabotics-2026.git
cd caos-lunabotics-2026
vcs import src < ros2.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## MVP on the Jetson (sensors + odometry, no RViz)

Use this when the Jetson runs the LiDAR stack and you view TF/scans from another machine (or CLI).

1. **Network / DDS** — On the Jetson (and on any laptop that should see the same topics), use the same domain and allow LAN discovery:

   ```bash
   export ROS_DOMAIN_ID=42
   export ROS_LOCALHOST_ONLY=0
   ```

2. **LiDAR** — In [`launch_jetson.py`](src/lunabotics_bringup/launch/launch_jetson.py), set `lidar_ip`, `local_ip`, and `serial_port` to match your L2 wiring (Ethernet and/or USB).

3. **Launch**

   ```bash
   source install/setup.bash
   ros2 launch lunabotics_bringup launch_jetson.py
   ```

   This starts: `base_link`→`unilidar_lidar` TF, Unitree node, `pointcloud_to_laserscan` → `/scan`, **rf2o** with **IMU fusion off** (publishes `odom`→`base_link` and `/odom_rf2o`), and `octomap_server`.

4. **RViz on a laptop** — Same `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY=0`, workspace sourced (or at least RViz + config path), then:

   ```bash
   ros2 launch lunabotics_bringup launch_local.py
   ```

   Optionally set `RVIZ_CONFIG_PATH` to your `view.rviz` path if the package is not installed on the laptop.

**Quick checks:** `ros2 topic echo /scan --once`, `ros2 topic echo /odom_rf2o --once`, `ros2 run tf2_ros tf2_echo odom base_link`.

## Other launches

| Launch | Role |
|--------|------|
| `launch.py` | Single machine: RealSense + LiDAR + EKF (IMU + rf2o) + depth scan + octomap + RViz |
| `launch_bench_mvp.launch.py` | LiDAR-only cart/bench test + rf2o + octomap + RViz (no camera) |
| `launch_TK.py` | Same as `launch.py` with alternate lidar→camera static TF |

## Hardware notes

See [`Description.md`](Description.md) for the intended Jetson / L2 / D435i layout.

## License

Apache-2.0 (see [LICENSE](LICENSE)).

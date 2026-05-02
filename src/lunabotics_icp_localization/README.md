# lunabotics_icp_localization

C++ ROS 2 node providing **LiDAR-only odometry** via Generalized ICP (GICP).
Replaces the previous `rf2o_laser_odometry` + `robot_localization` EKF chain.

Publishes:
- `/odometry/filtered` (`nav_msgs/Odometry`) — pose in the `odom` frame
- `odom → base_link` TF — consumed by Nav2, elevation_mapping, crater detection

> **TF timestamp note:** The TF transform is stamped with `this->now()` (wall-clock time at
> publication), not the LiDAR scan's header timestamp. This avoids a race with Nav2's
> message filter: if the TF were stamped at scan time, the costmap could receive the cloud
> before GICP finishes and see "timestamp earlier than all data in transform cache." The
> Nav2 costmaps are configured with `transform_tolerance: 0.5` to absorb the ≤GICP-cycle
> offset between TF stamp and scan stamp.

---

## How It Works

```
/unilidar/cloud ──► range clip (≤3 m) ──► VoxelGrid (5 cm) ──► GICP vs sliding submap
/unilidar/imu   ──► integrate ω, a ─────────────────────────► initial guess for GICP
                                                                     │
                                                                     ▼
                                                       current_pose_ (Eigen::Isometry3d)
                                                                     │
                                              ┌──────────────────────┤
                                              ▼                      ▼
                                    /odometry/filtered        odom→base_link TF
```

**Sliding submap:** The last N scans transformed into the world frame are merged and
re-voxelised (7 cm) to form the GICP target. This avoids full SLAM while giving enough
reference geometry for a 6.88 m arena traverse.

**IMU fallback:** If GICP does not converge (fitness > threshold), the pose advances using
the integrated IMU delta only.

---

## Competition Compliance

### Wall exclusion

The Lunabotics rulebook prohibits using arena walls for localization. This node enforces
that at the sensor level:

- Points beyond `icp_range_clip` (default **3.0 m**) are discarded **before** GICP.  
- The arena is 5.0 m wide — walls are never closer than ~2.5 m from centre.  
- At inspection, demonstrate: set `icp_range_clip: 2.5` and show GICP still converges
  with no wall returns in the submap.

### Compass / magnetometer

The rulebook requires the compass feature to be **disabled** on the Unitree L2 IMU.  
This node does not use magnetometer data, but the Unitree driver must be configured
to not publish it (or it must be zeroed in firmware) before competition.  
Carry vendor documentation or a screenshot of the firmware setting to the inspection.

---

## Parameters (`config/icp_localization.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_leaf_size` | 0.05 m | Downsample leaf size for current scan |
| `icp_range_clip` | 3.0 m | Max range for ICP input — excludes walls |
| `max_iterations` | 15 | GICP max iterations per frame |
| `max_correspondence_distance` | 1.0 m | GICP correspondence threshold |
| `fitness_threshold` | 0.8 | Score above which IMU fallback is used |
| `submap_size` | 8 | Number of past scans in sliding submap |
| `odom_frame` | `odom` | |
| `base_frame` | `base_link` | |
| `lidar_frame` | `unilidar_lidar` | |

---

## Running Standalone

```bash
source install/setup.bash
ros2 launch lunabotics_icp_localization icp_localization.launch.py
```

---

## Verification

```bash
# Push robot ~5 m by hand; pose should read ~5 m
ros2 topic echo /odometry/filtered --once

# TF should be live
ros2 run tf2_ros tf2_echo odom base_link

# Check no wall features in submap: drive near wall, fitness score should stay low
# (watch for WARN "GICP did not converge" in the node output)
```

---

## Files

| File | Description |
|------|-------------|
| `include/lunabotics_icp_localization/icp_localization_node.hpp` | Class declaration |
| `src/icp_localization_node.cpp` | Full implementation (~270 lines) |
| `config/icp_localization.yaml` | ROS 2 params |
| `launch/icp_localization.launch.py` | Launch file |

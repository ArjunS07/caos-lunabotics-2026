
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

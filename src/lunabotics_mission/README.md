# lunabotics_mission

Python ROS 2 package containing the **competition mission FSM** (Finite State Machine).
This is the top-level autonomy controller — it waits for the operator trigger, sends the
Nav2 goal to the Construction Zone, monitors progress, and announces completion.

---

## State Machine

```
IDLE ──(/autonomy_start service)──► TRAVERSING ──(Nav2 SUCCEEDED)──► ARRIVED
                                         │
                                         └──(Nav2 FAILED or timeout)──► retry goal
```

| State | What happens |
|-------|-------------|
| `IDLE` | Waiting. `/mission_state` publishes `"IDLE"` at 1 Hz. No Nav2 goals sent. |
| `TRAVERSING` | Nav2 goal active toward Construction Zone. Zone transitions logged to `rosout`. `/mission_state` publishes `"TRAVERSING"`. |
| `ARRIVED` | Nav2 reported success. `/mission_state` publishes `"ARRIVED"`. Operator announces completion to MCJ. |

---

## Triggering Autonomy

```bash
ros2 service call /autonomy_start std_srvs/srv/Trigger
```

Returns `success: True` if the node was in `IDLE` and Nav2 was reachable.

**Operator procedure before calling this:**
1. Place robot in the Excavation Zone
2. Rotate robot with remote control so its forward axis faces the Construction Zone
3. Verify `/odometry/filtered` shows near-zero pose (odom origin)
4. Announce to MCJ that you are going hands-free
5. Call `/autonomy_start`
6. Announce to MCJ that autonomy has begun (call returns success)

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/autonomy_start` | `std_srvs/Trigger` | service | Operator trigger |
| `/mission_state` | `std_msgs/String` | published | Current state string (IDLE / TRAVERSING / ARRIVED) |
| `/odometry/filtered` | `nav_msgs/Odometry` | subscribed | Used for zone distance monitoring |

---

## Zone Detection

Zone transitions use a **hybrid** approach (permitted by rulebook §Zone Detection):

1. **Distance prior** — Euclidean distance from odom origin. Past `obstacle_zone_distance`
   (default 2.5 m) → node logs "Entered Obstacle Zone confirmed".
2. **Sensor confirmation** — `/crater_detections` count can be checked as a secondary cue
   (not wired by default; add a subscription if needed).

This satisfies the rulebook requirement for *"Localization across the entire arena"*
without relying on predefined obstacle positions.

---

## Parameters (`config/mission.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `construction_zone_x` | 5.5 m | Nav2 goal X in odom frame |
| `construction_zone_y` | 1.25 m | Nav2 goal Y — offset toward berm wall |
| `obstacle_zone_distance` | 2.5 m | Distance threshold for Obstacle Zone detection log |
| `arrival_distance` | 5.0 m | Expected distance when reaching Construction Zone |
| `nav2_goal_timeout_s` | 120.0 s | Seconds before retrying the Nav2 goal |

**Venue tuning** — KSC and UCF arenas have different berm target positions:

| Venue | Suggested `construction_zone_x` | Suggested `construction_zone_y` |
|-------|--------------------------------|--------------------------------|
| UCF   | 5.5 m | 1.25 m |
| KSC   | 5.8 m | 1.5 m |

Update `config/mission.yaml` before each competition day.

---

## Running Standalone

```bash
source install/setup.bash
ros2 launch lunabotics_mission mission.launch.py
```

Requires Nav2 to be running (the action server `navigate_to_pose` must be available).

---

## Verification

```bash
# Watch state transitions
ros2 topic echo /mission_state

# Trigger autonomy (in a second terminal)
ros2 service call /autonomy_start std_srvs/srv/Trigger

# Expected output in node log:
#   State: IDLE → TRAVERSING
#   Sending Nav2 goal → (5.50, 1.25) in odom frame
#   Nav2 goal accepted
#   Entered Obstacle Zone (dist=2.5x m from origin)
#   *** AUTONOMY COMPLETE — announce to MCJ ***
#   State: TRAVERSING → ARRIVED
```

---

## Files

| File | Description |
|------|-------------|
| `lunabotics_mission/mission_node.py` | FSM + Nav2 action client |
| `config/mission.yaml` | ROS 2 params (tune per venue) |
| `launch/mission.launch.py` | Launch file |

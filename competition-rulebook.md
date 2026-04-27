# Lunabotics Travel Automation — Rules & Technical Reference

## Task Goal

Implement autonomous Travel Automation (250 pts) for the NASA Lunabotics Challenge. The robot must autonomously navigate from the Excavation/Starting Zone, through the Obstacle Zone, and into the Construction Zone. We are NOT implementing excavation or dumping — just the traverse.


## Arena Layout & Dimensions

Total interior: **6.88 m long × 5.0 m wide** (measured between interior ducts on the sides; ducts are 17 cm diameter).

The arena is filled with BP-1 (crushed basalt lunar regolith simulant) to ~45 cm depth. Random gravel (~2 cm diameter) may be mixed in.

### Zone Breakdown (along the 6.88 m length)

From the layout diagram, the arena is divided as follows:

| Zone | Length (along 6.88 m axis) | Notes |
|---|---|---|
| Starting Zone | ~2 m × 2 m box | Bottom-left corner of arena. Origin is (0,0,0°). Overlaps with Excavation Zone. No rocks or craters in the Starting Zone. |
| Excavation Zone | ~2.5 m (from the starting wall) | Full 5 m width. Overlaps with Starting Zone. Boulders MAY appear here but cannot exceed obstacle zone boulder dimensions. |
| Obstacle Zone | ~4.38 m (remainder of length) | Full 5 m width. Contains boulders and craters. Central support column (permanent obstacle) is in this zone. |
| Construction Zone | ~1.5 m deep area at the far end | Located at the opposite end from the starting zone, within the obstacle zone footprint. The berm target area is here. |

**Important:** The Construction Zone is drawn as an inset region at the far end of the arena (bottom-right in the layout diagram). It appears to occupy roughly the last ~1.5 m of the arena length and roughly half the width, positioned along one side wall. The berm target area (red box in diagram) is within this zone.

### Berm Target Dimensions

| | KSC | UCF |
|---|---|---|
| Berm Size | 2.0 m × 0.7 m | 1.3 m × 0.7 m |
| Target Berm Area (red box) | 2.2 m × 0.9 m | 1.5 m × 0.9 m |

UCF runs are 15 minutes. KSC runs are 30 minutes.

### Obstacles

**Boulders:**
- Minimum 3 in the obstacle zone
- 30–40 cm diameter, varying heights
- Randomly repositioned before each competition round
- May also appear in the excavation zone

**Craters:**
- Minimum 3 in the obstacle zone
- Varying depths and widths, not exceeding ~40–50 cm in either measurement

**Central support column:** Permanent obstacle in the middle of the obstacle zone. Must be avoided.

### Starting Conditions

- Robot is placed in a **randomly selected starting position and direction** within the Starting Zone.
- Direction is determined by spinning a direction wheel: the robot's declared forward arrow will face north, east, south, or west.
- The robot does NOT know its starting orientation in advance.
- No rocks or craters exist in the Starting Zone.


## Travel Automation Rules (250 pts)

### Requirements

1. The team must indicate to the MCJ (Mission Control Judge) that they are going hands-free **while still in the Excavation Zone**.
2. The robot must remain in hands-free mode while crossing the obstacle field and crossing into the Construction Zone.
3. The robot must demonstrate:
   - Localization across the entire competition arena
   - Object detection and location relative to the robot
   - Navigational planning based on location and obstacles/traversable area
4. Judges will set up the obstacle field to require a **"slalom" route** — a "point and traverse" (straight-line drive) approach is explicitly not allowed.

### Penalties

- **-30 pts (one-time):** If the robot contacts a rock or drives across a crater in the obstacle zone (as determined by MCJ/Arena judges).
- **-50 pts:** If the attempt is made AFTER the robot has already traversed the obstacle zone in remote control (to prevent "breadcrumbing" / learning the layout first). This penalty is assessed only once to the successful attempt.
- For maximum points, the attempt must be made at the **start of the run on the first time leaving the excavation zone**.

### Example Scoring

Robot crosses obstacle course in remote control before the attempt, hits an obstacle, and drives across a crater during the attempt: 250 - 50 - 30 = 170 points.


## Autonomy Rules (General — Apply to All Autonomy Tiers)

### Hands-Free Definition

"Hands-free" means ALL team members in the MCC must not touch any components (laptops, game controllers, etc.). The team MAY control the arena situational awareness camera(s) during this time.

### Judge Communication Protocol (CRITICAL — failure = 0 pts)

1. Clearly announce and **make eye contact** with the MCJ when going to autonomous operations.
2. Clearly announce when autonomy has **begun**.
3. Clearly announce when autonomy is **completed**.
4. If autonomy **fails**, announce the failure BEFORE resuming manual control.
5. If any team member touches any equipment while the robot is still moving or before autonomy is declared complete/failed, the attempt scores **zero points**.

### Prohibited During Autonomy

- Touching any controls in MCC.
- Transmitting orientation data or any control/influence signals to the robot.
- Using arena **walls** for mapping, navigation, or collision avoidance (no walls on the Moon).
- Touch sensors, spring wires, or any contact-based collision avoidance sensors against walls or surfaces.
- Updating or altering the autonomy program to account for / upload obstacle location information.
- Any real-time interaction that controls or influences the robot.

### Allowed During Autonomy

- Passive telemetry viewing (watching sensor data, no sending).
- Viewing an interface that shows telemetry data, as long as there is no interaction to control or influence the robot.
- Manipulating the NASA situational awareness cameras (MCC monitors).

### Disclosure Requirement

- Teams MUST explain to inspection judges how their autonomous systems work.
- Teams MUST prove that autonomy sensors do not use the walls.
- **Failure to divulge the method of autonomy sensing = disqualification.**


## Sensor & Equipment Restrictions

### Banned

- GPS or IMU-enabled GPS devices
- Compasses (analog, digital) — compass feature on IMUs must be switched off or data subtracted; must explain to judges
- Ultrasonic proximity sensors
- Touch sensors for obstacle sensing/avoidance
- Rubber pneumatic tires, air/foam filled tires, open or closed cell foam
- Hydraulics
- Wall-based sensing of any kind

### Allowed

- IMUs (with compass feature disabled/subtracted — must explain to judges how)
- LiDAR, cameras, encoders, wheel odometry
- Beacons or fiducial targets mounted on the bin frame structure along the **perimeter of the Starting Zone (2 sides only)**. Must be attached during setup time and removed afterward.
- Beacons can be passive fiducials, or active (signal/light/laser-based) — Class I or Class II lasers only (< 5 mW), must provide vendor documentation for eye safety.
- Beacon/fiducial mass counts toward the 80 kg robot mass limit and must be self-powered.
- Rods pushed into regolith at the starting area for beacon anchoring.

### Beacon Mounting Reference

From the document: beacon mounting coordinate reference is X: 5.38 m, Y: 0.6 m (context unclear without the figure — likely references a specific mounting point on the arena frame). Beacons can only be placed on the bin frame structure along the starting zone perimeter (2 sides).


## MCC (Mission Control Center) Constraints

- Max 4 team members in MCC. No faculty/advisors.
- No cell phones, smart devices, or extraneous electronics — only equipment required for robot operations.
- No external communications once in MCC (exception: communication with Arena teammates during setup via Lunabotics-provided radio only).
- Two situational awareness camera monitors provided in MCC. Use of these cameras during the run factors into the construction score.
- Operators may only use data and video originating from the robot and the competition video monitors.
- 30-minute competition run (KSC), 15-minute run (UCF).
- Robot must move within first 5 minutes or the run ends (robot considered inoperable).

### Mission Conclusion

- When timer expires, must send a command to stop the robot immediately.
- If the robot is mid-autonomous activity, must send an inhibit command.


## Robot Physical Constraints

- Payload envelope: 150 cm L × 75 cm W × 75 cm H (stowed). Orientation of dimensions is team's choice.
- May deploy/expand beyond envelope after start, but cannot exceed 175 cm additional height (250 cm total above regolith surface).
- Maximum mass: 80 kg (includes all subsystems, beacons, navigational aids).
- Minimum 4 designated lifting points.
- Robot is placed in arena in a randomly selected starting position AND direction (forward arrow faces N/E/S/W per direction wheel).


## Key Engineering Considerations for Travel Automation

### Localization Challenge

- Starting position and direction are random — the robot must determine its own pose at startup.
- Beacons/fiducials on the starting zone perimeter (2 sides) can serve as initial localization references.
- Must maintain localization across ~5–6 m of traverse through unknown obstacle placement.
- Cannot use walls for any sensing.
- No compass available.

### Obstacle Avoidance

- Must detect and navigate around boulders (30–40 cm) and craters (40–50 cm).
- Central column is a permanent obstacle.
- Obstacle placement is random and changes between rounds.
- Cannot pre-load obstacle positions.
- Must demonstrate a slalom-style navigation, not a straight drive.
- Contacting a rock or driving across a crater = -30 pts (one-time penalty).

### Zone Detection

- No explicit physical markers between zones that are robot-detectable.
- Zone transitions must be inferred from localization (distance traveled from known start) and/or environmental cues (presence/absence of obstacles).
- The Excavation Zone is ~2.5 m deep, then the Obstacle Zone runs ~4.38 m to the far end where the Construction Zone is.
- "Arriving" at the Construction Zone = having crossed through the obstacle field far enough (~5–6 m from the starting wall).

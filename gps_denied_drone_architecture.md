# GPS-Denied Multi-Drone Autonomous Platform — System Architecture

## 1. Project Vision

Build a fully autonomous GPS-denied single-drone system capable of detecting and mapping mines, maintaining at least 1 m safety clearance, identifying a safe route from Start Zone to Exit Zone, maintaining visual contact with a person, dynamically re-routing when necessary, and completing the mission within the competition time limit without manual intervention.

Long-term swarm objective: Prove reliable single-drone autonomy first and then extend the autonomy stack to multi-drone swarm operation.

---

## 2. Competition Mission Requirements

The drone must autonomously guide a person from a Start Zone to an Exit Zone through a minefield.

Competition target:
* Final minefield: 15 m × 60 m (configurable via ROS 2 parameters).
* Mission time limit: 10 minutes.
* GPS is NOT allowed.
* External computing systems are NOT allowed.
* Manual intervention is NOT allowed during autonomous operation.

**Configurable Environment Parameters**:
All environment dimensions (field length/width, Start Zone boundaries, Exit Zone boundaries) MUST be loaded at runtime from a ROS 2 parameter file (`mission_params.yaml`) rather than hardcoded in source nodes. This ensures easy adaptation between Stage 1 (~10m × 40m), Final (~15m × 60m), and custom simulation test fields.

The autonomous system must:
1. Detect mines.
2. Map detected mines.
3. Maintain at least a 1 m safety clearance around mines.
4. Detect and account for static obstacles.
5. Identify an unobstructed/safe route from Start Zone to Exit Zone.
6. Indicate or mark the safe route according to the competition's required mechanism.
7. Detect and maintain visual contact with the person.
8. Dynamically re-route when the person's movement or environmental conditions make the current route unsafe.
9. Perform autonomous takeoff.
10. Perform autonomous obstacle avoidance.
11. Complete the mission without manual input.

IMPORTANT:
Route marking/indication mechanism: TBD — competition specification confirmation required.
Competition compliance of the Raspberry Pi 5 must be confirmed against the competition's definition of "external computing systems." Do not automatically assume that Raspberry Pi 5 is allowed until the competition rules confirm that an onboard companion computer qualifies as permitted onboard computation.

---

## 3. Stage 1 Competition Simulation

Stage 1 is a smaller simulated version of the final problem.

Required environment:
* Approximately 10 m × 40 m field.
* At least 20 simulated mines.
* 3–5 static obstacles.
* Start Zone.
* Exit Zone.
* Simulated human/person.
* GPS unavailable.
* Fully autonomous operation.

Stage 1 minimum demonstration requirements:
* Autonomous takeoff.
* Autonomous obstacle avoidance.
* Detect at least 5 mines.
* Map detected mines.
* Maintain at least 1 m clearance from detected mines.
* Identify/indicate a safe path.
* Basic interaction with the human/person.
* No manual control during the autonomous demonstration.

The simulator must be a configurable simulation environment rather than hard-coding one exact mine layout.
The simulator should support:
* randomized mine positions,
* randomized obstacle positions,
* multiple test layouts,
* different person starting positions,
* person movement,
* repeatable test seeds,
* mission timer,
* ground-truth mine locations,
* ground-truth obstacle locations,
* ground-truth drone pose,
* ground-truth person position.

The Stage 1 environment must be treated as a competition validation environment, while `walls.sdf` can remain available as a generic localization/obstacle-development environment.

---

## 4. Core Design Rules & Architectural Principles

1. GPS is never required for the autonomy stack.
2. No remote/cloud/off-board computation may be required during competition operation.
3. All mission-critical autonomy must execute onboard the permitted flight/companion hardware.
4. Mine detection and generic obstacle detection are separate capabilities.
5. Mine mapping is separate from Visual-Depth localization.
6. A mine must generate an explicit safety exclusion zone.
7. The safe route planner must account for at least 1 m mine clearance.
8. The planner must account for localization and perception uncertainty.
9. Human tracking is a mission-critical autonomy subsystem.
10. Route planning must be dynamically re-plannable.
11. Manual intervention must not be required for the autonomous mission.
12. PX4 remains responsible for low-level flight control and safety.
13. ROS 2 remains responsible for perception, localization, planning and mission logic.
14. Avoid double-fusing correlated sensor measurements.
15. Stereo Depth Camera + IMU sensor suite provides complete 3D localization.
16. Keep all hardware-specific drivers behind clean ROS 2 interfaces.
17. Do not lock the architecture prematurely to a particular mine-detection ML algorithm or path-planning algorithm.
18. Every autonomous component must have a defined failure behavior.

---

## 5. Target Hardware

The platform uses specified physical hardware components while ensuring software abstractions remain modular:

### Flight Controller
**Darkmatter BRAHMA H7**:
* PX4 Autopilot.
* Low-level stabilization.
* Actuator control.
* Flight-controller failsafes.
* FC-side IMU.
* Barometer.
* State estimation handled through PX4 EKF2.

### ESC
**BotLab Dynamics 4-in-1 ESC**:
* Actuator subsystem.
* NOT a sensor.
* Exact current rating/protocol remains configurable.

### Depth Camera
**Intel RealSense D435i**:
* Primary role: depth perception.
* RGB perception.
* Point cloud generation.
* 3D obstacle perception.
* Mine detection/perception candidate source.
* Person detection/tracking candidate source.
* **Sensor Geometry & Simulation Assumption**: Use a configurable camera pose/FOV suitable for mine and obstacle observation in simulation. Exact physical mounting angle and hardware configuration will be validated during physical hardware integration.

Do NOT automatically fuse the D435i onboard IMU with the Brahma H7 IMU.

### Optical Flow
**Holybro PMW3901**:
* Ground-relative motion.
* Optical-flow velocity measurement.
* PX4 estimator input where supported.
* Keep custom ROS 2 processing optional.

### Stereo Depth Camera & IMU
**Intel RealSense D435i (Stereo Depth + IMU)**:
* 3D PointCloud & Depth image streams.
* Visual-Depth feature tracking & VIO pose estimation.
* High-rate IMU 6-DOF acceleration & angular velocity integration.
* 3D Visual SLAM.
* 3D obstacle geometry & mine mapping perception.

Stereo Depth Camera + IMU provides complete 3D perception, visual odometry, and obstacle mapping.

### Rangefinder
* **Status**: TBD.
* Required for reliable range/altitude support.
* Do not assume D435i or PMW3901 replaces a dedicated rangefinder.

### Companion Computer
**Raspberry Pi 5**:
* Planned onboard companion computer.
* **Competition compliance of the Raspberry Pi 5 must be confirmed against the competition's definition of 'external computing systems.' The architecture assumes computation is onboard the drone and not performed by a remote/cloud/off-board computer, subject to competition-rule confirmation.**

---

## 6. High-Level System Architecture & Data Flow

```text
Sensors
  ↓
Perception
  ├── GPS-Denied Localization
  ├── Mine Detection
  ├── Obstacle Detection
  └── Human Detection
          ↓
World Model
  ├── Drone Pose
  ├── Mine Map
  ├── Obstacle Map
  ├── Person State
  └── Free Space
          ↓
Safety Constraint Layer
  ├── 1m Mine Clearance
  ├── Obstacle Clearance
  └── Drone Footprint / Uncertainty
          ↓
Mission Planner
  ├── Safe Route
  ├── Route Indication
  ├── Human Guidance
  └── Dynamic Re-planning
          ↓
Offboard Control
          ↓
PX4
          ↓
Flight Controller / Motors
```

Ensure the diagram makes clear that PX4 remains responsible for low-level flight control and failsafes, while ROS 2 handles perception, localization, planning and mission autonomy.

### Coordinate Frame Transformations (TF2)
```text
field (Start Zone Origin: X=0, Y=0, Z=0)
 └── map
      └── odom
           └── base_link
                ├── imu_link
                ├── depth_camera_link (camera_color_optical_frame)
                ├── optical_flow_link
                └── rangefinder_link
```
- **Competition Field Reference (`field`)**: Fixed frame anchored at the Start Zone launch pad origin ($X=0, Y=0, Z=0$). All detected mine locations, static obstacle maps, and Exit Zone boundaries ($X=60\text{ m}$) are referenced to `field`.
- ROS 2 convention: **ENU / FLU** (East-North-Up / Forward-Left-Up).
- PX4 convention: **NED / FRD** (North-East-Down / Forward-Right-Down). Explicitly converted in state estimation odometry bridge.

---

## 7. Mission Autonomy Subsystems

The autonomy architecture distinguishes between collision avoidance and mission-critical mine and human tracking logic.

### Mission Execution Strategy Choices
The architecture supports two high-level mission state strategies:
1. **Scout-First Strategy (Recommended)**: Drone performs a fast initial mapping flight (1.5–2 min) over the 15m × 60m field to detect all mines and static obstacles and generate a globally optimal safe path *before* returning to the Start Zone to pick up and lead the human.
2. **Live Reconnaissance Strategy**: Drone leads the human continuously while mapping mines and static obstacles in real-time, dynamically re-routing whenever a blocked path is identified ahead.

### Mine Detection
Do NOT hard-code a specific ML algorithm.
Initially define the interface:
`MineDetection`:
* mine_id
* position
* confidence
* timestamp
* detection source
* observation status

Possible input sources:
* D435i RGB.
* D435i Stereo Depth & PointCloud.

The actual detector may later use classical computer vision, machine learning, deep learning, depth-based detection, or sensor fusion. Do not choose one permanently in the architecture unless explicitly required later.

The system must distinguish:
* candidate mine,
* confirmed mine,
* rejected false positive.

### Mine Map
The mine map must maintain:
* mine ID,
* estimated position,
* confidence,
* estimated size/radius where available,
* timestamp,
* detection status,
* safety zone.

Conceptual flow:
`Mine Detection` -> `Mine Confirmation` -> `Mine Localization` -> `Mine Map` -> `Safety Inflation` -> `Forbidden Region` -> `Path Planner`

Explicitly state that mine positions are expressed in the common localization/map coordinate frame.

### Safety Constraint Layer
Every confirmed mine must generate an exclusion region that guarantees at least 1 m clearance.

**Clearance Measurement Definition**:
Clearance is explicitly measured from the physical outer boundary of the mine to the physical outer collision footprint of the drone:
$$d_{\text{clearance}} = \|\mathbf{p}_{\text{drone}} - \mathbf{p}_{\text{mine}}\|_2 - r_{\text{drone}} - r_{\text{mine}} \ge 1.0\text{ m}$$
where:
* $\mathbf{p}_{\text{drone}}, \mathbf{p}_{\text{mine}}$: 2D positions of drone and mine centers in the `field` frame.
* $r_{\text{drone}}$: Drone bounding collision radius (~0.25 m).
* $r_{\text{mine}}$: Estimated mine physical radius (~0.15 m).

To guarantee safety despite localization drift and perception noise, the costmap inflation radius $R_{\text{inflation}}$ is defined as:
$$R_{\text{inflation}} = r_{\text{mine}} + 1.0\text{ m} + r_{\text{drone}} + \sigma_{\text{safety}}$$
where $\sigma_{\text{safety}}$ is an adaptive uncertainty buffer based on current EKF2 covariance and perception confidence. The planner treats $R_{\text{inflation}}$ as a hard obstacle cost.

### Static Obstacle Detection and Mapping
Input:
* D435i Stereo Depth Camera stream (`/camera/depth/image_raw`).
* PointCloud perception (`/camera/depth/color/points`).

Output:
* obstacle geometry,
* obstacle position,
* obstacle confidence,
* obstacle map.

The planner must consider both `Mine forbidden regions` + `Static obstacle forbidden regions` when generating a route.

### Minefield Search & Coverage Planner
Before or during human guidance, the system requires a coverage strategy to systematically observe the field:
* **Pattern Candidates**: Boustrophedon (lawnmower) sweep, spiral coverage, or frontier-based information gain search.
* **Configurable Parameters**: Sensor effective FOV width, sweep step size, altitude, flight speed.
* **Goal**: Maximize mine observation probability within time budget ($<2\text{ min}$ for scouting phase).

### Safe Route Planner
Inputs (Loaded dynamically via `mission_params.yaml`):
* Field dimensions (`field_length`, `field_width`).
* Start Zone bounds (`start_zone_x_min/max`, `start_zone_y_min/max`).
* Exit Zone bounds (`exit_zone_x_min/max`, `exit_zone_y_min/max`).
* Drone pose.
* Mine map & safety exclusion zones ($R_{\text{inflation}}$).
* Static obstacle map.
* Current person position & estimated trajectory.

Output:
* safe route corridor,
* route validity flag,
* route cost,
* trajectory setpoints.

The route planner must:
* avoid mines,
* maintain $\ge 1\text{ m}$ mine clearance (using $R_{\text{inflation}}$),
* avoid static obstacles,
* stay within field boundaries,
* dynamically re-plan upon path invalidation.

Prefer a modular planner architecture so the exact algorithm can be selected later (e.g. A*, RRT*, Nav2 costmap server, trajectory optimization).

### Safe Route Indication
The architecture must support the competition requirement that the drone identify/indicate a safe route.
Define:
Route indication mechanism: TBD.

Support two possible implementations until the competition rules are confirmed:
1. Physical route marking: requires a dedicated actuator/payload subsystem.
2. Digital/visual route indication: route is represented in the autonomy map/GCS visualization.

Do not assume which one is required. Add this as a hardware/rules checkpoint before final implementation.

### Human Detection & Tracking
Use the D435i RGB + depth as the primary candidate perception source.
Required outputs:
* person detected/not detected,
* person position,
* tracking confidence,
* timestamp,
* estimated movement direction/velocity where possible.

The system must maintain visual contact with the person during the mission.

**Human Guidance State Machine**:
* `IDLE_AT_START`: Hovering at Start Zone awaiting human presence.
* `LEADING_HUMAN`: Navigating 2–3 m ahead along the computed safe corridor.
* `WAIT_FOR_HUMAN`: If human-drone distance exceeds threshold ($>4\text{ m}$), hover in place and signal until human catches up.
* `HUMAN_OFF_PATH`: If human strays outside the safe 1 m clearance corridor, trigger warning signal and dynamically re-route safe waypoint to meet human.
* `EXIT_ARRIVED`: Safe arrival of person at Exit Zone ($X=60\text{ m}$); perform autonomous land sequence.

Distinguish:
* Person detection: "Where is the person?"
* Person tracking: "Where is the person now and where are they moving?"
* Person guidance: "What safe route should be maintained for the person?"

### Dynamic Mission Re-planning
The planner must continuously evaluate:
* current drone position,
* current person position,
* person movement,
* mine map updates,
* newly detected obstacles,
* current route validity.

If the current route becomes unsafe or unsuitable:
`Current Route` -> `Validity Check` -> `INVALID` -> `Recompute Safe Route` -> `Continue Mission`

The drone must not require manual intervention to re-plan.

---

## 8. Failure and Safety Architecture

Rather than blindly landing on recoverable errors, the system employs a **3-Tier Hierarchical Failure Recovery Model**:

1. **Tier 1: HOLD (In-Flight Pause)**
   - Freeze drone position setpoint, hold current altitude, pause mission execution state machine, and evaluate subsystem diagnostics for 3–5 seconds.
2. **Tier 2: RECOVER (Autonomous Re-alignment / Re-planning)**
   - Attempt automatic error recovery while airborne: re-acquire person tracking, re-compute alternative safe paths around new obstacles/mines, or fallback to secondary odometry sources.
3. **Tier 3: ABORT (Controlled Safe Flight Termination)**
   - Triggered ONLY if Tier 2 recovery fails or critical flight hardware/estimator safety checks fail (e.g., total localization loss, battery critical). Initiates controlled Return-to-Start or PX4 Autonomous Emergency Landing.

| Failure Condition | Tier 1 (HOLD) | Tier 2 (RECOVER) | Tier 3 (ABORT Fallback) |
| --- | --- | --- | --- |
| Mine detector failure | Pause motion | Rely on cached mine map & depth perception | ABORT: Return to Start Zone |
| False mine detection | Hold position | Re-evaluate spatial confidence over time | Dismiss detection if confidence < threshold |
| High localization uncertainty | Pause motion | Reduce flight velocity, increase safety buffer $\sigma_{\text{safety}}$ | ABORT: Land |
| Stereo Depth Camera failure | Hover in place | Fallback to Optical Flow / IMU Dead-Reckoning | ABORT: Safe Land |
| IMU failure | Hover in place | Rely on Stereo Depth Visual-Odom / Optical Flow | ABORT: Immediate Failsafe Land |
| Optical-flow failure | Hover in place | Rely on Stereo Depth Camera Visual Odometry | ABORT: PX4 Failsafe Land |
| Rangefinder failure | Hold altitude | Estimate height via D435i depth / IMU fusion | ABORT: Descend slowly & Land |
| Person tracking loss | Hover at last position | Perform 360° yaw search sweep to re-acquire | ABORT: Return to Start |
| Person route deviation | Hover in front of person | Re-compute safe corridor from human's current pose | ABORT: Trigger warning signal & Hover |
| Planner failure / Invalid path | Stop & Hover | Re-compute path with relaxed non-critical constraints | ABORT: Return to Start |
| DDS / Heartbeat loss | PX4 Offboard Hold | Auto-reconnect DDS agent within timeout | ABORT: PX4 Failsafe (RTL / Land) |
| PX4 Estimator failure | N/A | N/A | ABORT: Immediate PX4 Emergency Land |
| Low battery | N/A | N/A | ABORT: Return to Start / Land |
| Mission timeout (10 min) | N/A | N/A | ABORT: Controlled Landing |

*Note: In simulation and early testing, Tier 1 & Tier 2 recovery allows maximum debugging and learning without resetting the simulation on minor transient sensor glitches.*

---

## 9. Performance Metrics

The architecture defines measurable metrics:

**Localization**:
* position error,
* velocity error,
* yaw error,
* drift,
* covariance,
* latency.

**Mine detection**:
* detection count,
* detection precision/false positives,
* localization error,
* detection latency,
* minimum required detection count.

**Safety**:
* minimum mine distance,
* minimum obstacle distance,
* number of safety violations.

**Planning**:
* route length,
* route validity,
* planning time,
* re-planning time,
* successful Exit Zone arrival.

**Human tracking**:
* tracking confidence,
* percentage of mission with valid visual contact,
* person localization error,
* reacquisition time after temporary loss.

**Mission**:
* total mission time,
* autonomous completion rate,
* manual interventions,
* failsafe events.

---

## 10. Competition Success Criteria

### Stage 1 Target
| Requirement              | Stage 1 Target                        |
| ------------------------ | ------------------------------------- |
| Field size               | ~10 m × 40 m                          |
| Simulated mines          | >=20                                  |
| Mines detected           | >=5                                   |
| Static obstacles         | 3–5                                   |
| Mine clearance           | >=1 m                                 |
| Autonomous takeoff       | Required                              |
| Obstacle avoidance       | Required                              |
| Safe path identification | Required                              |
| Human interaction        | Required                              |
| Dynamic re-routing       | Required for full mission             |
| GPS                      | Not allowed                           |
| Manual input             | Not allowed                           |
| External computing       | Not allowed; exact interpretation TBD |
| Mission time             | Target <=10 min                       |

### Final Competition Target
| Requirement         | Final Target |
| ------------------- | ------------ |
| Field size          | 15 m × 60 m  |
| Mission time        | <=10 min     |
| GPS                 | Not allowed  |
| External computing  | Not allowed  |
| Manual intervention | Not allowed  |
| Mine detection      | Required     |
| Mine mapping        | Required     |
| Mine clearance      | >=1 m        |
| Safe route          | Required     |
| Person tracking     | Required     |
| Dynamic re-routing  | Required     |

---

## 11. Hardware Migration Strategy

1. **Development Order**: Phase 2 starts entirely in **SITL / Gazebo** simulation.
2. **Incremental Replacement**:
   Simulation -> sensor interface validation -> individual hardware sensor integration -> full onboard integration -> tethered flight -> autonomous flight.
3. Preserve standard ROS 2 interfaces between simulation and hardware wherever practical.
4. **Principle**: Software development does **NOT** wait for all physical hardware to arrive. Real-hardware work can begin incrementally whenever corresponding hardware is available.

---

## 12. Development Roadmap

### Phase 0 — Architecture and Interfaces
- Define ROS 2 interfaces, TF frames, sensor contracts, and PX4 boundary.
**Status: COMPLETE**

### Phase 1 — PX4 + ROS 2 Foundation
- Ubuntu 22.04, ROS 2 Humble, PX4 SITL, Gazebo, `px4_msgs`, Micro XRCE-DDS Agent.
**Status: COMPLETE**

### Phase 1.5 — Bidirectional PX4 Control
- ROS 2 Offboard node, heartbeat, setpoints, vehicle commands, telemetry monitoring, takeoff/hover/land sequence.
**Status: COMPLETE**

### Phase 2 — GPS-Denied Competition Autonomy
**Status: CURRENT**

#### Phase 2.1 — Competition Simulation Environment
* 10 m × 40 m field.
* 20+ mines (simulated via traffic cones).
* 3–5 static obstacles.
* Start Zone.
* Exit Zone.
* Human model.
* Mission timer.
* Ground-truth data.
* Randomized/repeatable scenarios.
**Status: COMPLETE**

#### Phase 2.2 — GPS-Denied Sensor & Localization Foundation
* High-rate IMU 6-DOF sensor.
* PMW3901 optical flow.
* Downward Rangefinder.
* Intel RealSense D435i Stereo Depth & RGB camera streams.
* PX4 EKF2 parameter profile (`EKF2_GPS_CTRL=0`, `EKF2_OF_CTRL=1`, `EKF2_HGT_MODE=2`).
* Diagnostic sensor health monitor node.
**Status: COMPLETE**

#### Phase 2.3 — Visual-Depth SLAM & Stereo Depth Mapping
* `/camera/depth/image_raw` + `/camera/depth/color/points` + IMU visual odometry.
* Visual-Depth SLAM in 3D PointCloud mode (0.05m grid cell resolution).
* 3D Occupancy Grid & PointCloud Map (`/map`) publication at 1 Hz.
* Dynamic transform publishing (`map -> odom` at 50 Hz).
* Complete TF tree: `map` -> `odom` -> `base_link` -> `camera_link`.
* Zero ground-truth input to SLAM/TF node.
**Status: COMPLETE**

#### Phase 2.4 — External Odometry & PX4 EKF2 Integration
* `px4_ext_odom_node` (`map -> base_link` TF conversion).
* ENU/FLU -> NED/FRD rigid matrix transformations.
* `VehicleOdometry` microsecond timestamp & schema formatting.
* `/fmu/in/vehicle_visual_odometry` bridge.
* EKF2 External Vision fusion (`EKF2_EV_CTRL=9`, `EKF2_HGT_REF=2`).
* Estimator health & innovation monitoring (`ev_hpos` innovations < 3mm).
**Status: COMPLETE**

#### Phase 2.5 — GPS-Denied Position Hold
* Autonomous takeoff.
* GPS-disabled hover.
* Position hold.
* Drift measurement.
* Controlled movement.
* Failsafe testing.

#### Phase 2.6 — Mine Detection & Mapping
* D435i perception.
* Mine detection.
* Mine confirmation.
* Mine localization.
* Mine map.
* Confidence tracking.
* False-positive handling.

#### Phase 2.7 — Safety Zones & Safe Route Planning
* 1 m mine safety zones.
* Obstacle inflation.
* Navigable-space representation.
* Start -> Exit route planning.
* Route validation.
* Route indication/marking interface.

#### Phase 2.8 — Human Detection & Tracking
* Person detection.
* Person localization.
* Person tracking.
* Visual-contact monitoring.
* Person movement estimation.

#### Phase 2.9 — Dynamic Re-routing & Autonomous Mission
* Person-aware planning.
* Dynamic mine-map updates.
* Dynamic obstacle updates.
* Route invalidation.
* Automatic re-planning.
* Autonomous guidance.
* Exit-zone arrival.
* Autonomous landing.

#### Phase 2.10 — Competition Validation & Optimization
Validate:
* 10-minute mission limit.
* At least 5 mine detections for Stage 1.
* 20+ mine environment.
* 3–5 static obstacles.
* >=1 m mine clearance.
* Autonomous takeoff.
* Autonomous obstacle avoidance.
* Safe-route identification.
* Human interaction.
* Dynamic re-routing.
* No GPS.
* No manual intervention.
* No prohibited external computing.
* CPU/RAM/thermal limits on onboard computation.

### Phase Dependencies
2.1 -> 2.2 -> 2.3 -> 2.4 -> 2.5 -> 2.6 -> 2.7 -> 2.8 -> 2.9 -> 2.10

*However, allow 2.6 Mine Detection and 2.8 Human Detection development to proceed in simulation independently once the required sensor simulation is available.*
*The complete mission in Phase 2.9 must not be considered valid until localization, mine mapping, safety constraints, path planning, and human tracking are all operational.*

---

## 13. Future Roadmap Phases

**The competition-specific single-drone autonomy stack must be validated before swarm development becomes the primary focus.**

#### Phase 3 — Aerostack2 Integration
- Localization integration, Aerostack2 planning, behaviors, mission execution.

#### Phase 4 — Raspberry Pi 5 + Pixhawk Hardware Integration
- ROS 2 Humble, Micro XRCE-DDS over UART (`TELEM2`), real sensor drivers, hardware calibration, thermal/CPU benchmarking.

#### Phase 5 — Real Single Drone Autonomous Flight
- Tethered tests, real localization, position hold, obstacle avoidance, failsafe validation on physical aircraft.

#### Phase 6 — Multi-Drone Simulation
- Multiple PX4 SITL instances, ROS 2 namespaces (`/drone_0`, `/drone_1`), Aerostack2 swarm coordination.

#### Phase 7 — Real Swarm
- Multiple physical drones, decentralized communication, formation control, collision avoidance.

#### Phase 8 — React Ground Control Station (GCS)
- React UI, FastAPI backend, rosbridge, telemetry visualization, mission control, diagnostics.

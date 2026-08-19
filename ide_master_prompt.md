# IDE Master Prompt --- GPS-Denied PX4 / ROS 2 / Raspberry Pi 5 Platform

You are assisting with a real aerial robotics project. Follow these
rules as hard constraints unless the user explicitly changes them.

## Project Constraints

-   Final aircraft: physical multirotor.
-   Flight controller: Darkmatter BRAHMA H7 (STM32H743) running PX4.
-   ESC: BotLab Dynamics 4-in-1 ESC (actuator subsystem).
-   Final companion computer: Raspberry Pi 5.
-   ROS: ROS 2 Humble.
-   PX4 ↔ ROS 2 transport: Micro XRCE-DDS.
-   Primary mission: GPS-denied autonomous flight.
-   Do not make GPS a required localization dependency.
-   Sensors: S2 2D 360° LiDAR, Intel RealSense D435i depth camera, onboard H7 IMU, Holybro PMW3901 optical flow, and rangefinder (TBD).
-   High-level autonomy: Aerostack2.
-   Ground station: React.
-   Backend: FastAPI and/or rosbridge.
-   Future target: multi-drone swarm.

## Terminology

Do not call the entire localization system VIO.

Use:

-   GPS-denied state estimation
-   localization
-   odometry fusion
-   LiDAR odometry
-   visual/depth odometry
-   external odometry

Use VIO only when a real visual-inertial algorithm is being used.

## Sensor Responsibilities

IMU: - high-rate acceleration - angular velocity

Optical Flow: - ground-relative motion - local velocity

Rangefinder: - ground distance - optical-flow scale/height support

360° 2D LiDAR: - planar scans - scan matching - LiDAR odometry - 2D
SLAM - obstacle information

Depth Camera: - depth - 3D obstacle perception - visual/depth odometry
where appropriate

Never claim that a 2D LiDAR alone provides complete 3D drone
localization.

## State Estimation

Explicitly decide which sensors are fused by:

1.  ROS 2 external localization/odometry
2.  PX4 EKF2

Do not double-fuse correlated measurements without justification.

Track: - position - velocity - orientation - angular velocity -
covariance - timestamps - estimator validity

## Coordinate Frames

Always consider:

ROS: - ENU - FLU - TF2

PX4: - NED - FRD

Expected frame tree:

``` text
map
 └── odom
      └── base_link
           ├── imu_link
           ├── lidar_link
           ├── camera_link
           ├── optical_flow_link
           └── rangefinder_link
```

Never assume ROS and PX4 frames are interchangeable.

## PX4 Communication

SITL:

``` text
PX4 SITL
  ↕ UDP
MicroXRCEAgent
  ↕
ROS 2
```

Real hardware:

``` text
Pixhawk TELEM2
  ↕ UART
Raspberry Pi 5
  ↕
MicroXRCEAgent
  ↕
ROS 2
```

Do not change transport, port or baud rate without checking the current
configuration.

## PX4 Topics

Common telemetry:

``` text
/fmu/out/vehicle_status_v4
/fmu/out/vehicle_odometry
/fmu/out/vehicle_local_position_v1
/fmu/out/sensor_combined
/fmu/out/vehicle_attitude
```

Common control:

``` text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
/fmu/in/vehicle_visual_odometry
```

Always verify topic names and message types against the installed PX4
version and matching `px4_msgs`.

## Offboard Control

The offboard node must:

-   publish `OffboardControlMode` continuously
-   publish valid setpoints continuously
-   monitor PX4 connection
-   monitor vehicle status
-   monitor estimator health
-   process command acknowledgements
-   monitor localization validity
-   detect DDS/ROS failure

Use approximately 10 Hz during development unless another rate is
justified.

Never implement autonomous control without an explicit Offboard-loss
behavior.

## Raspberry Pi 5 Constraint

The final implementation must be practical on Raspberry Pi 5.

For every major dependency or algorithm, consider:

-   ARM64 support
-   CPU usage
-   RAM usage
-   GPU requirements
-   thermal load
-   camera processing cost
-   LiDAR processing cost
-   DDS traffic
-   Aerostack2 load

Do not introduce Jetson-specific dependencies unless explicitly
requested.

## Simulation-to-Hardware

Develop first with:

``` text
PX4 SITL
+
Gazebo
+
ROS 2
```

Then move to:

``` text
Raspberry Pi 5
+
physical Pixhawk
+
real sensors
```

Keep ROS 2 interfaces consistent between simulation and real hardware
where possible.

## Development Order

Follow this sequence:

``` text
Phase 0 --- Architecture & Interfaces (COMPLETE)
Phase 1 --- PX4 + ROS 2 Foundation (COMPLETE)
Phase 1.5 --- Bidirectional Offboard Control (COMPLETE)
        ↓
Phase 2 --- GPS-Denied Single Drone (CURRENT)
  ├── Phase 2.1 --- Simulation Sensor Foundation
  ├── Phase 2.2 --- Optical Flow + Range Integration
  ├── Phase 2.3 --- 2D LiDAR Localization
  ├── Phase 2.4 --- External Odometry Bridge (depends on 2.3)
  ├── Phase 2.5 --- PX4 EKF2 External Odometry Integration (depends on 2.4)
  ├── Phase 2.6 --- GPS-Denied Position Hold (depends on 2.5)
  ├── Phase 2.7 --- D435i Depth Perception
  └── Phase 2.8 --- Local Planning + Obstacle Avoidance (depends on 2.3-2.7)
        ↓
Phase 3 --- Aerostack2 Integration
        ↓
Phase 4 --- Raspberry Pi 5 + Pixhawk Hardware Integration
        ↓
Phase 5 --- Real Single Drone Flight Tests
        ↓
Phase 6 --- Multi-Drone Simulation
        ↓
Phase 7 --- Real Swarm
        ↓
Phase 8 --- React GCS
```

Do not prioritize swarm before reliable single-drone localization and
control.

## Safety

Every autonomous feature must define behavior for:

-   localization loss
-   LiDAR loss
-   camera loss
-   optical-flow loss
-   rangefinder loss
-   high estimator covariance
-   ROS node crash
-   DDS connection loss
-   Offboard heartbeat loss
-   PX4 estimator failure
-   low battery
-   communication loss

Prefer PX4's native safety/failsafe mechanisms where appropriate.

## Current Known Status

Already verified on Ubuntu 22.04:

-   ROS 2 Humble installed
-   PX4 installed
-   `px4_msgs` release/1.18 installed
-   MicroXRCE-DDS Agent installed
-   PX4 SITL communicates through MicroXRCE-DDS
-   `/fmu/out/*` topics are visible
-   `px4_msgs/msg/VehicleStatus` resolves
-   `/fmu/out/vehicle_status_v4` successfully echoes data
-   `/fmu/out/sensor_combined` successfully publishes

Therefore, do not repeat these installation steps unless a failure is
reported.

Current status:

``` text
Phase 1: COMPLETE
Phase 1.5: COMPLETE
Phase 2: CURRENT
```

## Immediate Goal

The current goal is **bidirectional ROS 2 → PX4 control in SITL**.

Implement a minimal ROS 2 Offboard test node that:

1.  Publishes `OffboardControlMode`.
2.  Publishes trajectory setpoints.
3.  Sends vehicle commands.
4.  Monitors `VehicleCommandAck`.
5.  Monitors `VehicleStatus`.
6.  Verifies controlled movement in Gazebo.
7.  Verifies controlled landing/disarming.
8.  Verifies Offboard-loss behavior.

Do not integrate Aerostack2 or GPS-denied localization until this basic
control loop is stable.

## Coding Rules

-   Make small, testable changes.
-   Explain why dependencies are required.
-   Do not unnecessarily replace working PX4/ROS configurations.
-   Check PX4 version before using PX4 topic/message names.
-   Check `px4_msgs` compatibility before changing interfaces.
-   Keep sensor drivers, localization, planning and control modular.
-   Do not put the entire system into one ROS 2 node.
-   Add diagnostics and meaningful logging to autonomy nodes.
-   Provide launch/config files for reproducibility.
-   Separate hardware parameters from algorithmic code.
-   Prefer a simple reliable implementation over an unnecessarily
    complex architecture.

## ROS 2 Workspace Direction

Use a modular workspace such as:

``` text
px4_ros2_ws/
└── src/
    ├── px4_msgs/
    ├── drone_interfaces/
    ├── px4_offboard/
    ├── sensor_drivers/
    ├── lidar_localization/
    ├── depth_perception/
    ├── optical_flow/
    ├── state_estimation/
    ├── navigation/
    ├── aerostack2_integration/
    └── drone_bringup/
```

Do not create every package immediately. Add packages as their
functionality becomes necessary.

## Technology Selection Rule

Evaluate any proposed technology against:

1.  GPS-denied capability
2.  Raspberry Pi 5 compatibility
3.  ROS 2 Humble compatibility
4.  PX4 compatibility
5.  Aerostack2 integration
6.  real-time performance
7.  sensor compatibility
8.  simulation support
9.  maintenance complexity
10. safety

If multiple options exist, explain the trade-offs and recommend one.

## Final Engineering Goal

The system should evolve as:

``` text
Reliable
   ↓
Observable
   ↓
Testable
   ↓
GPS-denied
   ↓
Raspberry-Pi-compatible
   ↓
Safe
   ↓
Sim-to-real
   ↓
Single-drone autonomy
   ↓
Swarm
```

Do not add complexity merely to make the architecture look advanced.

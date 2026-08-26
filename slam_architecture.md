# 🛸 Swarm SLAM & Autonomous System Architecture

This document provides a comprehensive overview of the **System Architecture**, **2D LiDAR SLAM Pipeline**, **Swarm Control Flow**, and the **Hardware Deployment Roadmap** (including Aerostack2 integration choices).

---

## 1. Executive Summary & Flow Diagram

The system operates in a **GPS-Denied Environment** using **2D LiDAR SLAM** (`slam_toolbox`) combined with **PX4 EKF2 Sensor Fusion** (Optical Flow + Rangefinder) for multi-drone autonomous minefield mapping, path planning, verification, and human escort.

```mermaid
flowchart TD
    %% Subtle Color Styling
    classDef sensors fill:#e1f5fe,stroke:#0288d1,stroke-width:1.5px,color:#01579b;
    classDef bridge fill:#fff3e0,stroke:#f57c00,stroke-width:1.5px,color:#e65100;
    classDef est fill:#e8f5e9,stroke:#388e3c,stroke-width:1.5px,color:#1b5e20;
    classDef autonomy fill:#f3e5f5,stroke:#7b1fa2,stroke-width:1.5px,color:#4a148c;

    subgraph Sensors["SENSORS & GAZEBO SIMULATION"]
        Lidar["2D LiDAR (/scan)"]
        Range["Downward Rangefinder"]
        OF["Optical Flow"]
    end

    subgraph Middleware["COMMUNICATION BRIDGE"]
        GZBridge["ros_gz_bridge"]
        XRCE["MicroXRCE-DDS Agent"]
    end

    subgraph Localization["LOCALIZATION & ESTIMATION"]
        SLAM["slam_toolbox (2D SLAM)"]
        TFPub["sim_tf_publisher (odom -> base_link)"]
        ExtOdom["px4_ext_odom_node"]
        EKF2["PX4 EKF2 State Estimator"]
    end

    subgraph Autonomy["SWARM AUTONOMY LAYER"]
        SwarmNode["swarm_mission_node (Scouts 0,1,2)"]
        PathPlanner["safe_path_planner (A* + Merge)"]
        Verifier["verifier_drone_controller (Drone 3)"]
        Walker["human_walker_node"]
    end

    Sensors --> GZBridge
    Sensors --> XRCE
    GZBridge --> SLAM
    GZBridge --> TFPub
    SLAM --> ExtOdom
    ExtOdom --> XRCE
    XRCE --> EKF2
    EKF2 --> SwarmNode
    SwarmNode --> PathPlanner
    PathPlanner --> Verifier
    Verifier --> Walker

    class Sensors,Lidar,Range,OF sensors;
    class Middleware,GZBridge,XRCE bridge;
    class Localization,SLAM,TFPub,ExtOdom,EKF2 est;
    class Autonomy,SwarmNode,PathPlanner,Verifier,Walker autonomy;
```

---

## 2. Current Simulation Architecture Breakdown

### A. Localization & Mapping Pipeline (`slam_toolbox`)
* **Package**: `slam_toolbox` (`async_slam_toolbox_node`)
* **Algorithm**: Graph-based 2D LiDAR SLAM with Karto optimizer.
* **Input**: 2D Laser Scans (`/scan`, `/model/x500_lidar_2d_N/scan`).
* **Frame Hierarchy**:
  $$\text{map} \longrightarrow \text{odom} \longrightarrow \text{base\_link} \longrightarrow \text{lidar\_link}$$

### B. PX4 Sensor Fusion (EKF2 Integration)
To achieve stable 3D flight from a 2D LiDAR:
1. **$X, Y$ Position & Yaw**: Derived from 2D LiDAR SLAM pose via `px4_ext_odom_node` fed into `VehicleOdometry` uORB topic.
2. **$Z$ Altitude**: Measured directly by Downward Rangefinder / Distance Sensor.
3. **Horizontal Velocity**: Estimated via Optical Flow sensor.

### C. Swarm Control & Mission Pipeline
* **Scouts (Drones 0, 1, 2)**: Sweep South ($Y=-2$), Center ($Y=0$), and North ($Y=+2$) lanes simultaneously.
* **Path Planner (`safe_path_planner`)**: Merges scout paths using A* grid search into a single **Human Escape Corridor** that maximizes clearance from detected mine positions.
* **Verifier Drone (Drone 3)**: Escorts along the escape corridor, performs low-altitude clearance checks at each waypoint, and publishes a `SAFE`/`UNSAFE` verdict report.
* **Human Escort (`human_walker_node`)**: Animates human model along the verified corridor once safety is confirmed.

---

## 3. Hardware Deployment Options: Current Stack vs. Aerostack2

When transitioning from **Gazebo Simulation** to **Real Physical Hardware** (e.g., NXP B3RB / Jetson Orin Nano + PX4 Autopilot + RPLiDAR A2/A3):

### Option A: Retain Native Custom ROS 2 Stack (Recommended for Initial Hardware Tests)
* **Pros**:
  * 100% code reusability from simulation to real drones.
  * Direct low-latency control via `px4_msgs` over MicroXRCE-DDS (Serial/UART bridge).
  * Lightweight footprint running directly on companion computers (Raspberry Pi 4 / Jetson Orin Nano).
* **Cons**: Manual configuration of companion-to-PX4 serial parameters.

### Option B: Migrate to Aerostack2 (AS2) Framework
* **Pros**:
  * Standardization: Uses standard ROS 2 Actions (`Takeoff`, `Land`, `GoTo`, `FollowPath`).
  * Behavior Trees: Easily reconfigurable mission logic.
  * Multi-robot fleet management interface.
* **Cons**:
  * High migration effort (requires wrapping custom minefield path merger into AS2 plugin modules).
  * Higher memory and computational overhead on small companion boards.

---

## 4. Hardware Comparison Matrix

| Feature | Current Sim Stack (`robofest_sim`) | Hardware via Native ROS 2 | Hardware via Aerostack2 (AS2) |
| :--- | :--- | :--- | :--- |
| **Transport** | MicroXRCE-DDS (UDP / Sim) | MicroXRCE-DDS (UART/Serial) | AS2 Platform Driver for PX4 |
| **SLAM** | `slam_toolbox` (Async) | `slam_toolbox` / Cartographer | AS2 SLAM / Localization Node |
| **Mission Logic** | Custom Python State Machine | Custom Python State Machine | AS2 Behavior Trees |
| **Migration Risk**| None (Baseline) | **Low** (Change UDP $\to$ TTY) | **High** (Refactor to AS2 Actions) |

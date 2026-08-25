#!/usr/bin/env python3
"""
swarm_mission_node.py  —  3-Drone Swarm Mission

Controls 3 PX4 drones simultaneously inside one ROS 2 node.
Each drone runs its own state-machine (DroneController) and publishes
to its own namespaced PX4 topics.

Coordinate frame reference
--------------------------
  Gazebo ENU  : X = East (field length, 0→40 m)
                Y = North (field width, -5→+5 m)
  PX4 NED local (per drone, from its own spawn home):
                X = North = GazeboY − spawnGazeboY
                Y = East  = GazeboX − spawnGazeboX   (= GazeboX − 4.5 for all)

Drone spawn positions (Gazebo)
------------------------------
  Drone 0  →  (4.5, -2, 0.25)   south lane
  Drone 1  →  (4.5,  0, 0.25)   centre lane  (existing single-drone path)
  Drone 2  →  (4.5, +2, 0.25)   north lane

Formation landing (blue-box, Gazebo X=37.5)
-------------------------------------------
  Drone 0  →  (37.5, -2)
  Drone 1  →  (37.5,  0)
  Drone 2  →  (37.5, +2)
  All achieved by commanding PX4 local (x=0, y=33) on each drone.
"""

import math
import json
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleStatus, VehicleLocalPosition,
)

# Verifier drone (lives in robofest_sim package, same workspace)
from robofest_sim.verifier_drone_controller import VerifierDroneController, VerifierConfig

# ─────────────────────────────────────────────────────────────────────────────
# Tunable constants
# ─────────────────────────────────────────────────────────────────────────────
FLIGHT_ALT        = -1.2          # NED altitude (negative = up)
WP_RADIUS         = 0.6           # metres — waypoint acceptance radius
HOLD_TICKS        = 40            # ticks × 0.1 s = 4 s hold at each station
TAKEOFF_HOLD_TICKS = 40           # 4 s stabilisation at green box after takeoff
ARM_RETRY_LIMIT   = 60            # 60 × 10 ticks × 0.1s = 60 s before abort
WARMUP_TIMEOUT    = 600           # 600 ticks = 60 s max warmup per drone
MAX_SPEED_MPS     = 0.5           # m/s maximum horizontal speed
TICK_RATE_HZ      = 10.0          # Hz control rate

# ─────────────────────────────────────────────────────────────────────────────
# Per-drone configuration
# ─────────────────────────────────────────────────────────────────────────────
#   namespace  : ROS 2 topic prefix  ("" → /fmu/...,  "px4_1" → /px4_1/fmu/...)
#   waypoints  : list of (px4_x, px4_y) in NED from home
#   mid_wp_idx : index after which we enter HOLD_MID (drone pauses at that WP)
#
# All three paths share the same PX4 Y offsets (East = field-length axis) but
# differ in PX4 X (North = cross-field axis) so they never collide.
# ─────────────────────────────────────────────────────────────────────────────
DRONE_CONFIGS = [
    {   # ── Drone 0 ─ SOUTH LANE ───────────────────────────────────────────
        # spawn Gazebo (4.5, -2) → avoids Tree1(18,+2) & Tree2(23,-2) by going south
        "id":        0,
        "namespace": "",          # instance 0  →  /fmu/...
        "waypoints": [
            (-1.0,  7.5),   # WP0 approach      Gazebo (12,  -3)
            (-2.0, 12.5),   # WP1 past Tree1    Gazebo (17,  -4)  [tree at Y=+2 → 6m clear]
            (-1.0, 16.5),   # WP2 mid-hold      Gazebo (21,  -3)
            (-1.5, 21.5),   # WP3 past Tree2    Gazebo (26, -3.5) [tree at Y=-2 → 1.5m, ok at 1.2m alt]
            ( 0.0, 33.0),   # WP4 blue box      Gazebo (37.5,-2)
        ],
        "mid_wp_idx": 2,
    },
    {   # ── Drone 1 ─ CENTRE LANE ──────────────────────────────────────────
        # spawn Gazebo (4.5, 0)  → weaves around both trees
        "id":        1,
        "namespace": "px4_1",    # instance 1  →  /px4_1/fmu/...
        "waypoints": [
            ( 0.0,  9.5),   # WP0 approach      Gazebo (14,  0)
            (-2.5, 13.5),   # WP1 past Tree1    Gazebo (18, -2.5) [tree at Y=+2 → 4.5m clear]
            ( 0.0, 16.0),   # WP2 mid-hold      Gazebo (20.5, 0)
            ( 2.5, 18.5),   # WP3 past Tree2    Gazebo (23, +2.5) [tree at Y=-2 → 4.5m clear]
            ( 0.0, 33.0),   # WP4 blue box      Gazebo (37.5, 0)
        ],
        "mid_wp_idx": 2,
    },
    {   # ── Drone 2 ─ NORTH LANE ───────────────────────────────────────────
        # spawn Gazebo (4.5, +2) → avoids both trees by going north
        "id":        2,
        "namespace": "px4_2",    # instance 2  →  /px4_2/fmu/...
        "waypoints": [
            ( 1.0,  9.5),   # WP0 approach      Gazebo (14, +3)
            ( 2.0, 13.5),   # WP1 past Tree1    Gazebo (18, +4)   [tree at Y=+2 → 2m clear north]
            ( 1.0, 16.5),   # WP2 mid-hold      Gazebo (21, +3)
            ( 1.0, 21.5),   # WP3 clear north   Gazebo (26, +3)
            ( 0.0, 33.0),   # WP4 blue box      Gazebo (37.5,+2)
        ],
        "mid_wp_idx": 2,
    },
]


# ─────────────────────────────────────────────────────────────────────────────
class DroneController:
    """
    Single-drone state machine.
    All publishers / subscribers are owned by the parent ROS 2 Node;
    this class only stores drone-specific state and business logic.
    """

    STATES = ["WARMUP", "ARM", "TAKEOFF", "NAVIGATE",
              "HOLD_MID", "HOLD_END", "LAND", "DONE"]

    def __init__(self, node: Node, config: dict, qos: QoSProfile):
        self.node       = node
        self.drone_id   = config["id"]
        self.waypoints  = config["waypoints"]
        self.mid_wp_idx = config["mid_wp_idx"]
        ns              = config["namespace"]

        # Topic prefix  (e.g. ""  →  "/fmu/...",  "px4_1" → "/px4_1/fmu/...")
        prefix = f"/{ns}" if ns else ""

        self.offboard_pub = node.create_publisher(
            OffboardControlMode,
            f"{prefix}/fmu/in/offboard_control_mode", qos)
        self.trajectory_pub = node.create_publisher(
            TrajectorySetpoint,
            f"{prefix}/fmu/in/trajectory_setpoint", qos)
        self.cmd_pub = node.create_publisher(
            VehicleCommand,
            f"{prefix}/fmu/in/vehicle_command", qos)

        node.create_subscription(
            VehicleStatus,
            f"{prefix}/fmu/out/vehicle_status_v4",
            self._status_cb, qos)
        node.create_subscription(
            VehicleLocalPosition,
            f"{prefix}/fmu/out/vehicle_local_position_v1",
            self._pos_cb, qos)

        # State
        self.state         = "WARMUP"
        self.nav_state     = 0
        self.arming_state  = 0
        self.cur_x = self.cur_y = self.cur_z = 0.0
        self.sp_x = self.sp_y = 0.0  # Current setpoint for interpolation
        self.sp_z = FLIGHT_ALT        # Tracks current Z setpoint, starts at flight alt
        self.state_timer   = 0
        self.wp_idx        = 0
        self.ekf2_ready    = False   # set by swarm node
        # Per-drone readiness: wait for first status message
        self.has_status    = False   # True once _status_cb fires at least once
        self.has_pos       = False   # True once _pos_cb fires at least once

    # ── ROS callbacks ────────────────────────────────────────────────────────
    def _status_cb(self, msg):
        self.nav_state    = msg.nav_state
        self.arming_state = msg.arming_state
        self.has_status   = True

    def _pos_cb(self, msg):
        self.cur_x = msg.x
        self.cur_y = msg.y
        self.cur_z = msg.z
        self.has_pos = True

    # ── Publish helpers ──────────────────────────────────────────────────────
    def _pub_offboard(self):
        msg = OffboardControlMode()
        msg.timestamp  = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.position   = True
        msg.velocity   = False
        msg.acceleration = False
        msg.attitude   = False
        msg.body_rate  = False
        self.offboard_pub.publish(msg)

    def _pub_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.position  = [float(x), float(y), float(z)]
        msg.velocity  = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw       = float('nan')
        self.trajectory_pub.publish(msg)

    def _send_cmd(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp         = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.command           = command
        msg.param1            = float(p1)
        msg.param2            = float(p2)
        msg.target_system     = 1 + self.drone_id   # MAVLink sys ID 1, 2, 3
        msg.target_component  = 1
        msg.source_system     = 255
        msg.source_component  = 1
        msg.from_external     = True
        self.cmd_pub.publish(msg)

    def _log(self, msg, level="info"):
        prefix = f"[Drone{self.drone_id}]"
        if level == "warn":
            self.node.get_logger().warn(f"{prefix} {msg}")
        elif level == "error":
            self.node.get_logger().error(f"{prefix} {msg}")
        else:
            self.node.get_logger().info(f"{prefix} {msg}")

    def _transition(self, new_state):
        self._log(f"{self.state} → {new_state}")
        self.state       = new_state
        self.state_timer = 0

    # ── Main tick (called at 10 Hz by the parent node) ───────────────────────
    def tick(self):
        if self.state == "DONE":
            return

        self._pub_offboard()

        # ── WARMUP / ARM ─────────────────────────────────────────────────────
        if self.state in ("WARMUP", "ARM"):
            self._pub_setpoint(0.0, 0.0, FLIGHT_ALT)  # hold at home

            if self.state == "WARMUP":
                # Wait for actual status data from this drone specifically
                if self.has_status and self.has_pos and self.ekf2_ready:
                    self._log(f"Status received and EKF2 ready — transitioning to ARM")
                    self._transition("ARM")
                elif self.state_timer >= WARMUP_TIMEOUT:
                    self._log(f"Warmup timeout ({WARMUP_TIMEOUT} ticks) — forcing ARM", "warn")
                    self._transition("ARM")
                else:
                    if self.state_timer % 50 == 0:  # log every 5s
                        self._log(f"Waiting for DDS data... "
                                  f"status={self.has_status} pos={self.has_pos} "
                                  f"({self.state_timer//10}s)")
                    self.state_timer += 1

            else:  # ARM
                if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    if self.state_timer % 10 == 0:
                        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    if self.state_timer % 50 == 0:
                        self._log(f"Waiting for OFFBOARD mode... nav_state={self.nav_state} "
                                  f"({self.state_timer//10}s)")
                elif self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    if self.state_timer % 10 == 0:
                        attempt = self.state_timer // 10
                        if attempt >= ARM_RETRY_LIMIT:
                            self._log("Arm retries exceeded — aborting.", "error")
                            self._transition("DONE")
                        else:
                            self._send_cmd(
                                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
                    if self.state_timer % 50 == 0:
                        self._log(f"Waiting for ARM... arming_state={self.arming_state} "
                                  f"({self.state_timer//10}s)")
                else:
                    self._transition("TAKEOFF")
                self.state_timer += 1

        # ── TAKEOFF ──────────────────────────────────────────────────────────
        elif self.state == "TAKEOFF":
            self.sp_x, self.sp_y = 0.0, 0.0
            self._pub_setpoint(self.sp_x, self.sp_y, FLIGHT_ALT)
            if abs(self.cur_z - FLIGHT_ALT) < 0.25:
                if self.state_timer >= TAKEOFF_HOLD_TICKS:   # 4 s green-box hold
                    self._transition("NAVIGATE")
                self.state_timer += 1
            else:
                self.state_timer = 0

        # ── NAVIGATE ─────────────────────────────────────────────────────────
        elif self.state == "NAVIGATE":
            if self.wp_idx >= len(self.waypoints):
                self._transition("HOLD_END")
                return

            tx, ty = self.waypoints[self.wp_idx]
            
            # Interpolate setpoint to limit speed
            max_step = MAX_SPEED_MPS / TICK_RATE_HZ
            dx = tx - self.sp_x
            dy = ty - self.sp_y
            dist_to_wp = math.hypot(dx, dy)
            
            if dist_to_wp > max_step:
                self.sp_x += (dx / dist_to_wp) * max_step
                self.sp_y += (dy / dist_to_wp) * max_step
            else:
                self.sp_x = tx
                self.sp_y = ty

            self._pub_setpoint(self.sp_x, self.sp_y, FLIGHT_ALT)

            # Check if drone physically reached the waypoint
            dist_physical = math.hypot(self.cur_x - tx, self.cur_y - ty)
            if dist_physical < WP_RADIUS and dist_to_wp <= max_step:
                self._log(f"Reached WP {self.wp_idx + 1}/{len(self.waypoints)}")
                self.wp_idx += 1

                # Enter mid-field hold right after mid_wp_idx waypoint
                if self.wp_idx == self.mid_wp_idx + 1:
                    self._transition("HOLD_MID")

        # ── HOLD_MID ─────────────────────────────────────────────────────────
        elif self.state == "HOLD_MID":
            tx, ty = self.waypoints[self.mid_wp_idx]
            self._pub_setpoint(tx, ty, FLIGHT_ALT)
            self.state_timer += 1
            if self.state_timer >= HOLD_TICKS:
                self._transition("NAVIGATE")

        # ── HOLD_END ─────────────────────────────────────────────────────────
        elif self.state == "HOLD_END":
            tx, ty = self.waypoints[-1]
            self._pub_setpoint(tx, ty, FLIGHT_ALT)
            self.state_timer += 1
            if self.state_timer >= HOLD_TICKS:
                self.sp_z = FLIGHT_ALT   # seed descent from current flight altitude
                self._transition("LAND")

        # ── LAND ─────────────────────────────────────────────────────────────
        elif self.state == "LAND":
            tx, ty = self.waypoints[-1]
            # Ascend setpoint Z from FLIGHT_ALT (e.g. -1.5 NED) toward -0.05 (ground).
            # FLIGHT_ALT is negative in NED, so adding a positive rate raises it.
            descent_rate = 0.03   # 0.03 m/tick × 10 Hz = 0.3 m/s descent
            self.sp_z = min(-0.05, self.sp_z + descent_rate)
            self._pub_setpoint(tx, ty, self.sp_z)
            self.state_timer += 1

            # Send NAV_LAND once on entry
            if self.state_timer == 1:
                self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

            # Once setpoint is near ground, start issuing disarm commands
            if self.sp_z >= -0.15:
                if self.state_timer % 10 == 0:
                    self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=0.0)

            # Done: (a) PX4 disarmed, (b) near ground + long wait, (c) hard timeout safety
            actually_on_ground = abs(self.cur_z) <= 0.4
            if (self.arming_state == VehicleStatus.ARMING_STATE_DISARMED
                    or (self.state_timer > 120 and actually_on_ground)
                    or self.state_timer > 200):
                self._transition("DONE")
                self._log("=== Scout Mission Complete & Landed ===")


    @property
    def is_done(self):
        return self.state == "DONE"


# ─────────────────────────────────────────────────────────────────────────────
class SwarmMissionNode(Node):
    """Orchestrates 3 scout DroneControllers + 1 VerifierDroneController."""

    # Default manifest path — override via ROS 2 parameter
    _MANIFEST_DEFAULT = (
        '/home/ubuntu/px4_ros2_ws/src/robofest_sim/worlds/generated/stage1_manifest.json'
    )

    def __init__(self):
        super().__init__('swarm_mission_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Scout drones (unchanged) ──────────────────────────────────────────
        self.drones = [DroneController(self, cfg, qos) for cfg in DRONE_CONFIGS]

        # ── Load mine manifest for verifier clearance checks ─────────────────
        self.declare_parameter('manifest_path', self._MANIFEST_DEFAULT)
        manifest_path = self.get_parameter('manifest_path').value
        mines = self._load_mines(manifest_path)

        # ── Verifier drone (Drone 3) ──────────────────────────────────────────
        verifier_cfg = VerifierConfig(
            drone_id         = 3,
            namespace        = 'px4_3',
            flight_alt_ned   = -1.5,     # slightly higher than scouts at -1.2
            cruise_speed_mps = 1.0,
            wp_radius_m      = 0.5,
            dwell_ticks      = 30,       # 3 s dwell at each WP
            min_clearance_m  = 1.0,
        )
        self._verifier = VerifierDroneController(self, verifier_cfg, qos, mines)

        # Subscribe to shared EKF2 readiness signal
        self.create_subscription(Bool, '/ekf2_ready', self._ekf2_cb, 10)

        # Publisher to notify when at least 2 scouts have landed
        self.scouts_done_pub = self.create_publisher(Bool, '/mission/scouts_done', qos)

        self.get_logger().info(
            f"SwarmMissionNode started — {len(self.drones)} scouts + 1 verifier. "
            "Waiting for EKF2 readiness..."
        )

        self._tick_timer = self.create_timer(0.1, self._tick)

    @staticmethod
    def _load_mines(path: str) -> list:
        """Load mine specs from scenario manifest. Returns [] on any error."""
        try:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    data = json.load(f)
                mines = data.get('mines', [])
                return mines
        except Exception as e:
            pass
        return []

    def _ekf2_cb(self, msg: Bool) -> None:
        for d in self.drones:
            d.ekf2_ready = msg.data
        self._verifier.ekf2_ready = msg.data

    def _tick(self) -> None:
        # Tick all scout drones
        for d in self.drones:
            d.tick()

        scouts_done_count = sum(1 for d in self.drones if d.is_done)
        all_scouts_done = (scouts_done_count == len(self.drones))
        at_least_2_scouts_done = (scouts_done_count >= 2)

        # Notify planner when at least 2 scouts have landed (triggers map display)
        msg = Bool()
        msg.data = at_least_2_scouts_done
        self.scouts_done_pub.publish(msg)

        # Tick verifier — gates on ALL scouts finishing
        self._verifier.tick(scouts_all_done=all_scouts_done)

        # Shut down only when verifier is also done
        if all_scouts_done and self._verifier.is_done:
            self.get_logger().info(
                "=== ALL DRONES + VERIFIER COMPLETE — Mission finished. ===")
            self._tick_timer.cancel()


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SwarmMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

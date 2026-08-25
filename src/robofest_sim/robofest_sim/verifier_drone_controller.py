#!/usr/bin/env python3
"""
verifier_drone_controller.py
=============================
State machine for Drone 3 (the Verifier).

Responsibilities (Single Responsibility Principle):
  - Wait on the ground until all 3 scout drones finish (state: WAIT_SCOUTS).
  - Arm, take off, and fly to each human-path waypoint (state: VERIFY).
  - Hover at each waypoint for a configurable dwell time to simulate
    a sensor check (camera/LiDAR cross-validation).
  - Validate physical clearance from all mines at each waypoint using
    ClearanceValidator (Open/Closed: swap validator without touching state machine).
  - Publish a detailed verification report (state: REPORT) then land.

Integration:
  - Imported and instantiated by SwarmMissionNode (same Node, zero extra processes).
  - Reads human path from /planning/human_path (nav_msgs/Path).
  - Publishes verdict to /mission/verification_report (std_msgs/String).
  - Publishes progress markers to /mission/verifier_markers.

Design notes:
  - Does NOT subclass DroneController: the verifier has a fundamentally
    different state graph. Composition over inheritance.
  - All PX4 I/O is still done through the same pattern: publishers on the
    parent Node so there is only one executor.
  - mines list is injected at construction (Dependency Inversion Principle).
"""

import math
import json
import os
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleStatus, VehicleLocalPosition,
)

# Pure-function validator — no ROS dependency
from robofest_sim.path_corridor import ClearanceValidator


# ─────────────────────────────────────────────────────────────────────────────
# Internal state enum (private to this module)
# ─────────────────────────────────────────────────────────────────────────────
class _VState:
    WAIT_SCOUTS = "WAIT_SCOUTS"   # Idle on ground, watching scout completion
    ARM         = "ARM"           # Arm + set OFFBOARD
    TAKEOFF     = "TAKEOFF"       # Climb to verifier altitude
    VERIFY      = "VERIFY"        # Fly to WP_N, dwell, record verdict
    REPORT      = "REPORT"        # Publish final verdict
    CLEAR_PATH  = "CLEAR_PATH"    # Fly out of the human path to avoid blocking
    LAND        = "LAND"          # Issue LAND command
    DONE        = "DONE"          # Terminal


# ─────────────────────────────────────────────────────────────────────────────
# Configuration dataclass for the verifier drone
# ─────────────────────────────────────────────────────────────────────────────
class VerifierConfig:
    """
    All tunable parameters for the verifier drone.
    Keeps magic numbers in one place and makes testing easy.
    """
    def __init__(
        self,
        drone_id:         int   = 3,
        namespace:        str   = "px4_3",
        flight_alt_ned:   float = -1.5,     # NED (negative = up). Slightly higher than scouts.
        cruise_speed_mps: float = 0.5,      # m/s setpoint interpolation speed
        wp_radius_m:      float = 0.5,      # metres to consider a WP reached
        dwell_ticks:      int   = 30,       # 10 Hz ticks to hover at each WP (3 s)
        arm_retry_limit:  int   = 60,
        warmup_timeout:   int   = 200,
        min_clearance_m:  float = 1.0,
        tick_rate_hz:     float = 10.0,
    ):
        self.drone_id         = drone_id
        self.namespace        = namespace
        self.flight_alt_ned   = flight_alt_ned
        self.cruise_speed_mps = cruise_speed_mps
        self.wp_radius_m      = wp_radius_m
        self.dwell_ticks      = dwell_ticks
        self.arm_retry_limit  = arm_retry_limit
        self.warmup_timeout   = warmup_timeout
        self.min_clearance_m  = min_clearance_m
        self.tick_rate_hz     = tick_rate_hz


# ─────────────────────────────────────────────────────────────────────────────
# Main verifier drone controller
# ─────────────────────────────────────────────────────────────────────────────
class VerifierDroneController:
    """
    4th drone: verifies the merged human escape path, then publishes verdict.

    Parameters
    ----------
    node       : rclpy.node.Node   Parent ROS 2 node (owns timers + executor).
    config     : VerifierConfig    Tunable parameters.
    qos        : QoSProfile        Shared QoS matching PX4 DDS settings.
    mines      : list              Mine specs from scenario manifest.
    """

    def __init__(
        self,
        node:   Node,
        config: VerifierConfig,
        qos:    QoSProfile,
        mines:  list,
    ):
        self._node   = node
        self._cfg    = config
        self._mines  = mines
        self._state  = _VState.WAIT_SCOUTS
        self._timer  = 0

        # PX4 positional state (NED, relative to home)
        self._cur_x = self._cur_y = self._cur_z = 0.0
        self._sp_x  = self._sp_y = 0.0           # current interpolated setpoint
        self._nav_state    = 0
        self._arming_state = 0
        self._has_status   = False
        self._has_pos      = False
        self.ekf2_ready    = False                # set by SwarmMissionNode

        # Human path (world-frame ENU waypoints)
        # Coordinate conversion: NED_x = Gazebo_Y − spawnY, NED_y = Gazebo_X − spawnX
        self._human_wps_ned: List[Tuple[float, float]] = []  # converted to NED
        self._human_wps_world: List[Tuple[float, float]] = []  # ENU for clearance checks
        self._wp_idx         = 0
        self._verdicts: List[bool] = []           # True/False per waypoint
        self._dwell_timer    = 0

        # ── ROS topic prefix ─────────────────────────────────────────────────
        ns     = config.namespace
        prefix = f"/{ns}" if ns else ""

        # ── PX4 command publishers ────────────────────────────────────────────
        self._offboard_pub = node.create_publisher(
            OffboardControlMode,
            f"{prefix}/fmu/in/offboard_control_mode", qos)
        self._traj_pub = node.create_publisher(
            TrajectorySetpoint,
            f"{prefix}/fmu/in/trajectory_setpoint", qos)
        self._cmd_pub = node.create_publisher(
            VehicleCommand,
            f"{prefix}/fmu/in/vehicle_command", qos)

        # ── PX4 state subscribers ─────────────────────────────────────────────
        node.create_subscription(
            VehicleStatus,
            f"{prefix}/fmu/out/vehicle_status_v4",
            self._status_cb, qos)
        node.create_subscription(
            VehicleLocalPosition,
            f"{prefix}/fmu/out/vehicle_local_position_v1",
            self._pos_cb, qos)

        # ── Human path subscriber (from safe_path_planner) ────────────────────
        plan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5)
        node.create_subscription(
            Path, '/planning/human_path', self._human_path_cb, plan_qos)

        # ── Result publishers ─────────────────────────────────────────────────
        self._report_pub  = node.create_publisher(String,       '/mission/verification_report', 10)
        self._marker_pub  = node.create_publisher(MarkerArray,  '/mission/verifier_markers',    plan_qos)

        self._log("VerifierDroneController ready. State: WAIT_SCOUTS")

    # ─────────────────────────────────────────────────────────────────────────
    # ROS callbacks
    # ─────────────────────────────────────────────────────────────────────────
    def _status_cb(self, msg: VehicleStatus) -> None:
        self._nav_state    = msg.nav_state
        self._arming_state = msg.arming_state
        self._has_status   = True

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._cur_x    = msg.x
        self._cur_y    = msg.y
        self._cur_z    = msg.z
        self._has_pos  = True

    def _human_path_cb(self, msg: Path) -> None:
        """Receive merged human path from safe_path_planner."""
        if not msg.poses:
            return
        # Only update if we haven't started verifying yet
        if self._state not in (_VState.WAIT_SCOUTS, _VState.ARM):
            return

        self._human_wps_world = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in msg.poses
        ]
        # Convert world ENU → PX4 NED.
        # Spawn: Gazebo (2.5, 1.0)
        # NED_x = Gazebo_Y − 1.0,  NED_y = Gazebo_X − 2.5
        spawn_gaz_x, spawn_gaz_y = 2.5, 1.0
        self._human_wps_ned = [
            (wy - spawn_gaz_y, wx - spawn_gaz_x)      # (NED_x, NED_y)
            for (wx, wy) in self._human_wps_world
        ]
        self._log(f"Received human path: {len(self._human_wps_ned)} waypoints")

    # ─────────────────────────────────────────────────────────────────────────
    # PX4 publish helpers (same pattern as DroneController)
    # ─────────────────────────────────────────────────────────────────────────
    def _pub_offboard(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp    = int(self._node.get_clock().now().nanoseconds / 1000)
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self._offboard_pub.publish(msg)

    def _pub_setpoint(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp    = int(self._node.get_clock().now().nanoseconds / 1000)
        msg.position     = [float(x), float(y), float(z)]
        msg.velocity     = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw          = float('nan')
        self._traj_pub.publish(msg)

    def _send_cmd(self, command: int, p1: float = 0.0, p2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp        = int(self._node.get_clock().now().nanoseconds / 1000)
        msg.command          = command
        msg.param1           = float(p1)
        msg.param2           = float(p2)
        msg.target_system    = 1 + self._cfg.drone_id
        msg.target_component = 1
        msg.source_system    = 255
        msg.source_component = 1
        msg.from_external    = True
        self._cmd_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Setpoint interpolation (speed-limited like scout drones)
    # ─────────────────────────────────────────────────────────────────────────
    def _step_setpoint_toward(self, tx: float, ty: float) -> Tuple[float, float]:
        """Move self._sp_x/y one tick toward (tx,ty) at cruise speed."""
        max_step = self._cfg.cruise_speed_mps / self._cfg.tick_rate_hz
        dx = tx - self._sp_x
        dy = ty - self._sp_y
        dist = math.hypot(dx, dy)
        if dist > max_step:
            self._sp_x += (dx / dist) * max_step
            self._sp_y += (dy / dist) * max_step
        else:
            self._sp_x, self._sp_y = tx, ty
        return self._sp_x, self._sp_y

    # ─────────────────────────────────────────────────────────────────────────
    # Logging helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _log(self, msg: str, level: str = "info") -> None:
        prefix = f"[Verifier/Drone{self._cfg.drone_id}]"
        logger = self._node.get_logger()
        if level == "warn":
            logger.warn(f"{prefix} {msg}")
        elif level == "error":
            logger.error(f"{prefix} {msg}")
        else:
            logger.info(f"{prefix} {msg}")

    def _transition(self, new_state: str) -> None:
        self._log(f"{self._state} → {new_state}")
        self._state = new_state
        self._timer = 0

    # ─────────────────────────────────────────────────────────────────────────
    # Verification logic (pure, injected mines list — Dependency Inversion)
    # ─────────────────────────────────────────────────────────────────────────
    def _check_clearance_at_current_wp(self) -> bool:
        """
        Return True if current WP (world frame) clears all mines by min_clearance.
        Uses the pure ClearanceValidator from path_corridor.py.
        """
        if self._wp_idx >= len(self._human_wps_world):
            return False
        wx, wy = self._human_wps_world[self._wp_idx]
        clr = ClearanceValidator.clearance_at(wx, wy, self._mines)
        passed = clr >= self._cfg.min_clearance_m
        self._log(
            f"WP{self._wp_idx} ({wx:.2f},{wy:.2f}): clearance={clr:.2f}m "
            f"{'✓ PASS' if passed else '✗ FAIL'}"
        )
        return passed

    def _build_final_report(self) -> str:
        total = len(self._verdicts)
        passed = sum(self._verdicts)
        failed = total - passed
        overall = "SAFE" if all(self._verdicts) else "UNSAFE"
        lines = [
            f"=== VERIFIER REPORT === [{overall}]",
            f"Waypoints checked: {total} | PASS: {passed} | FAIL: {failed}",
        ]
        for idx, v in enumerate(self._verdicts):
            if idx < len(self._human_wps_world):
                wx, wy = self._human_wps_world[idx]
                clr = ClearanceValidator.clearance_at(wx, wy, self._mines)
                lines.append(
                    f"  WP{idx:02d} ({wx:5.2f},{wy:5.2f}): "
                    f"{'PASS' if v else 'FAIL'} | clearance={clr:.2f}m"
                )
        if overall == "SAFE":
            lines.append("HUMAN MAY PROCEED from START to EXIT. ✓")
        else:
            lines.append("PATH UNSAFE — verification failed. Manual re-plan required.")
        return "\n".join(lines)

    # ─────────────────────────────────────────────────────────────────────────
    # RViz2 marker publisher for live progress
    # ─────────────────────────────────────────────────────────────────────────
    def _publish_progress_markers(self) -> None:
        now  = self._node.get_clock().now().to_msg()
        ma   = MarkerArray()
        life = Duration(); life.sec = 3

        for idx, (wx, wy) in enumerate(self._human_wps_world):
            if idx >= len(self._verdicts):
                color_rgba = (0.7, 0.7, 0.7, 0.6)   # grey = pending
            elif self._verdicts[idx]:
                color_rgba = (0.0, 1.0, 0.3, 1.0)   # green = PASS
            else:
                color_rgba = (1.0, 0.1, 0.0, 1.0)   # red = FAIL

            m = Marker()
            m.header.frame_id    = 'world'
            m.header.stamp       = now
            m.ns                 = 'verifier_progress'
            m.id                 = idx
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.lifetime           = life
            m.pose.position.x    = wx
            m.pose.position.y    = wy
            m.pose.position.z    = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.4
            m.scale.z            = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = color_rgba
            ma.markers.append(m)

        # Verifier drone body marker (current NED → world approx)
        spawn_gaz_x, spawn_gaz_y = 2.5, 1.0
        drone_world_x = self._cur_y + spawn_gaz_x
        drone_world_y = self._cur_x + spawn_gaz_y
        dm = Marker()
        dm.header.frame_id    = 'world'
        dm.header.stamp       = now
        dm.ns                 = 'verifier_drone'
        dm.id                 = 9999
        dm.type               = Marker.SPHERE
        dm.action             = Marker.ADD
        dm.lifetime           = life
        dm.pose.position.x    = drone_world_x
        dm.pose.position.y    = drone_world_y
        dm.pose.position.z    = abs(self._cur_z)
        dm.pose.orientation.w = 1.0
        dm.scale.x = dm.scale.y = dm.scale.z = 0.5
        dm.color.r = 1.0; dm.color.g = 0.5; dm.color.b = 0.0; dm.color.a = 1.0  # orange
        ma.markers.append(dm)

        self._marker_pub.publish(ma)

    # ─────────────────────────────────────────────────────────────────────────
    # Main tick — called at TICK_RATE_HZ by SwarmMissionNode._tick()
    # ─────────────────────────────────────────────────────────────────────────
    def tick(self, scouts_all_done: bool) -> None:
        """
        Parameters
        ----------
        scouts_all_done : bool
            True when all 3 scout drones have reached DONE state.
            Gate that triggers the verifier's ARM sequence.
        """
        if self._state == _VState.DONE:
            return

        # Always keep offboard heartbeat alive once past WAIT_SCOUTS
        if self._state != _VState.WAIT_SCOUTS:
            self._pub_offboard()

        # ── WAIT_SCOUTS ───────────────────────────────────────────────────────
        if self._state == _VState.WAIT_SCOUTS:
            if scouts_all_done and self._human_wps_ned:
                self._log("All scouts DONE and human path received — beginning verification.")
                self._verdicts = []
                self._wp_idx   = 0
                self._transition(_VState.ARM)
            elif self._timer % 50 == 0:
                self._log(
                    f"Waiting... scouts_done={scouts_all_done} "
                    f"path_wps={len(self._human_wps_ned)} ({self._timer//10}s)",
                    "info"
                )
            self._timer += 1
            return

        # ── ARM ───────────────────────────────────────────────────────────────
        if self._state == _VState.ARM:
            self._pub_setpoint(0.0, 0.0, self._cfg.flight_alt_ned)
            if self._nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if self._timer % 10 == 0:
                    self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif self._arming_state != VehicleStatus.ARMING_STATE_ARMED:
                if self._timer % 10 == 0:
                    if self._timer // 10 >= self._cfg.arm_retry_limit:
                        self._log("ARM retry limit exceeded — aborting.", "error")
                        self._transition(_VState.DONE)
                        return
                    self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            else:
                self._transition(_VState.TAKEOFF)
            self._timer += 1
            return

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        if self._state == _VState.TAKEOFF:
            self._sp_x, self._sp_y = 0.0, 0.0
            self._pub_setpoint(0.0, 0.0, self._cfg.flight_alt_ned)
            if abs(self._cur_z - self._cfg.flight_alt_ned) < 0.3:
                if self._timer >= 30:   # 3 s stabilisation
                    self._log("Reached cruise altitude. Starting verification sweep.")
                    self._transition(_VState.VERIFY)
                self._timer += 1
            else:
                self._timer = 0
            return

        # ── VERIFY ────────────────────────────────────────────────────────────
        if self._state == _VState.VERIFY:
            if self._wp_idx >= len(self._human_wps_ned):
                self._transition(_VState.REPORT)
                return

            tx_ned, ty_ned = self._human_wps_ned[self._wp_idx]
            self._step_setpoint_toward(tx_ned, ty_ned)
            self._pub_setpoint(self._sp_x, self._sp_y, self._cfg.flight_alt_ned)

            dist_physical = math.hypot(self._cur_x - tx_ned, self._cur_y - ty_ned)
            dist_sp       = math.hypot(tx_ned - self._sp_x, ty_ned - self._sp_y)

            if dist_physical < self._cfg.wp_radius_m and dist_sp <= (
                self._cfg.cruise_speed_mps / self._cfg.tick_rate_hz
            ):
                # Drone is on-station — dwell phase
                self._dwell_timer += 1
                if self._dwell_timer >= self._cfg.dwell_ticks:
                    verdict = self._check_clearance_at_current_wp()
                    self._verdicts.append(verdict)
                    self._publish_progress_markers()
                    self._dwell_timer = 0
                    self._wp_idx += 1
                    if self._wp_idx < len(self._human_wps_ned):
                        self._log(f"Moving to WP {self._wp_idx}/{len(self._human_wps_ned)}")
            return

        # ── REPORT ────────────────────────────────────────────────────────────
        if self._state == _VState.REPORT:
            if self._timer == 0:
                report = self._build_final_report()
                self._log("\n" + report)
                msg = String(); msg.data = report
                self._report_pub.publish(msg)
                self._publish_progress_markers()
            self._timer += 1
            if self._timer >= 20:   # 2 s to let subscribers receive the report
                self._log("Verification complete. Clearing human path before landing...")
                self._transition(_VState.CLEAR_PATH)
            return

        # ── CLEAR_PATH ────────────────────────────────────────────────────────
        if self._state == _VState.CLEAR_PATH:
            # Target clear zone: Gazebo (38.0, -4.0) (bottom right of exit zone)
            # Avoids Drone 0 at (37.5, -2.0), Drone 1 at (37.5, 0), Drone 2 at (37.5, 2)
            # Avoids blocking the human path.
            # Convert to NED for Drone 3 (Spawn: Gazebo(2.5, 1.0))
            # NED_x = Gazebo_Y - 1.0 = -4.0 - 1.0 = -5.0
            # NED_y = Gazebo_X - 2.5 = 38.0 - 2.5 = 35.5
            tx_ned, ty_ned = -5.0, 35.5

            self._step_setpoint_toward(tx_ned, ty_ned)
            self._pub_setpoint(self._sp_x, self._sp_y, self._cfg.flight_alt_ned)

            dist_physical = math.hypot(self._cur_x - tx_ned, self._cur_y - ty_ned)
            if dist_physical < self._cfg.wp_radius_m:
                self._log("Reached clear landing zone. Safe for human!")
                self._transition(_VState.LAND)
            return

        # ── LAND ──────────────────────────────────────────────────────────────
        if self._state == _VState.LAND:
            if self._timer == 0:
                self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._timer += 1
            if self._arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self._transition(_VState.DONE)
                self._log("=== Verifier Mission Complete ===")

    @property
    def is_done(self) -> bool:
        return self._state == _VState.DONE

    @property
    def is_active(self) -> bool:
        """True once the verifier has started its active verification sweep."""
        return self._state == _VState.VERIFY

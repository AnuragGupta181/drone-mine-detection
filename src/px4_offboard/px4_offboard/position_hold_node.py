#!/usr/bin/env python3
"""
position_hold_node.py — Phase 2.5 GPS-Denied Obstacle Avoidance Mission

Executes a rigid state machine to validate GPS-denied flight.
Reads EKF2 readiness, arms cleanly, takes off to 2.5m, flies through waypoints
to avoid trees, holds position at the blue box, and lands.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand, 
                          VehicleStatus, VehicleCommandAck, VehicleLocalPosition)

class DriftTracker:
    def __init__(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y
        self.errors = []
        self.start_time = None
        self.settling_time = None
        self.threshold = 0.5

    def add_sample(self, current_time_sec, x, y):
        if self.start_time is None:
            self.start_time = current_time_sec
            
        error = math.sqrt((x - self.target_x)**2 + (y - self.target_y)**2)
        self.errors.append(error)
        
        if self.settling_time is None and error < self.threshold:
            self.settling_time = current_time_sec - self.start_time

    def generate_report(self):
        if not self.errors:
            return "No drift data collected."
        
        mean_err = sum(self.errors) / len(self.errors)
        max_err = max(self.errors)
        rms_err = math.sqrt(sum(e**2 for e in self.errors) / len(self.errors))
        final_err = self.errors[-1]
        settling = f"{self.settling_time:.2f}s" if self.settling_time is not None else "Did not settle"
        
        return (f"Mean: {mean_err:.3f}m, Max: {max_err:.3f}m, RMS: {rms_err:.3f}m, "
                f"Final: {final_err:.3f}m, Settling: {settling}")

class PositionHoldNode(Node):
    def __init__(self):
        super().__init__('position_hold_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_callback, qos_profile)
        self.ready_sub = self.create_subscription(Bool, '/ekf2_ready', self.ready_callback, 10)

        # State Variables
        self.state = "WARMUP"
        self.ekf2_ready = False
        self.nav_state = 0
        self.arming_state = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.state_timer = 0
        self.warmup_timeout = 300 # 30 seconds at 10Hz
        
        self.drift_trackers = {}
        self.current_tracker = None

        # Waypoints in PX4 LOCAL NED frame (X=North, Y=East from home)
        # Gazebo ENU: field runs along Gazebo X (East). Drone spawns at Gazebo (4.5, 0).
        # PX4 NED X = Gazebo Y (North, cross-field direction)
        # PX4 NED Y = Gazebo X - 4.5 (East, along the field length)
        #
        # Gazebo positions:  (X=18,Y=+2) = Tree1,  (X=23,Y=-2) = Tree2
        # Blue box center at Gazebo X=37.5, Y=0
        #
        # Formula: px4_x = gazebo_y, px4_y = gazebo_x - 4.5 (home offset)
        self.waypoints = [
            (0.0,  9.5),   # WP 0: Approach  — Gazebo (14.0,  0.0)
            (-2.5, 13.5),  # WP 1: Round Tree1 — Gazebo (18.0, -2.5) [tree at Y=+2]
            (0.0,  16.0),  # WP 2: Mid field  — Gazebo (20.5,  0.0) — HOLD HERE
            (2.5,  18.5),  # WP 3: Round Tree2 — Gazebo (23.0, +2.5) [tree at Y=-2]
            (0.0,  33.0),  # WP 4: Blue box   — Gazebo (37.5,  0.0)
        ]
        self.current_wp_idx = 0
        self.flight_altitude = -1.2

        self.get_logger().info("Position Hold Mission Node Started. Awaiting EKF2 Readiness...")
        self.timer = self.create_timer(0.1, self.timer_callback)

    def ready_callback(self, msg):
        self.ekf2_ready = msg.data

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def local_pos_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        
        if self.current_tracker is not None:
            if self.state in ["HOLD_END", "HOLD_MID"]:
                self.current_tracker.add_sample(self.get_clock().now().nanoseconds / 1e9, self.current_x, self.current_y)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan') # Let the drone decide heading based on velocity
        self.trajectory_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)

    def transition_to(self, new_state):
        self.get_logger().info(f"Transitioning: {self.state} -> {new_state}")
        self.state = new_state
        self.state_timer = 0

    def timer_callback(self):
        # Always stream offboard heartbeat
        if self.state not in ["LAND", "DONE"]:
            self.publish_offboard_control_mode()

        # Home tracking in PX4 NED frame (starts at 0,0 from PX4 home)
        if not hasattr(self, 'home_x'):
            self.home_x = 0.0  # PX4 NED X at spawn = 0 (North from home)
            self.home_y = 0.0  # PX4 NED Y at spawn = 0 (East from home)

        if self.state in ["WARMUP", "ARM"]:
            # Keep home locked at origin; don't drift track during warmup
            pass
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.flight_altitude)
            
            if self.state == "WARMUP":
                if self.ekf2_ready or self.state_timer > 30:
                    self.transition_to("ARM")
                else:
                    self.state_timer += 1
            elif self.state == "ARM":
                if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    if self.state_timer % 10 == 0:
                        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                elif self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                    if self.state_timer % 10 == 0:
                        attempts = self.state_timer // 10
                        if attempts > 3:
                            self.get_logger().error("Failed to arm cleanly. Aborting.")
                            self.transition_to("DONE")
                        else:
                            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
                else:
                    self.transition_to("TAKEOFF")
                self.state_timer += 1

        elif self.state == "TAKEOFF":
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.flight_altitude)
            if abs(self.current_z - (self.flight_altitude)) < 0.25:
                # Hold at the start zone (greenbox) for 4 seconds
                if self.state_timer > 40:
                    self.transition_to("NAVIGATE")
                self.state_timer += 1
            else:
                self.state_timer = 0

        elif self.state == "NAVIGATE":
            if self.current_wp_idx >= len(self.waypoints):
                self.transition_to("HOLD_END")
                return
                
            target_x, target_y = self.waypoints[self.current_wp_idx]
            self.publish_trajectory_setpoint(target_x, target_y, self.flight_altitude)
            
            dist = math.sqrt((self.current_x - target_x)**2 + (self.current_y - target_y)**2)
            if dist < 0.6:
                self.get_logger().info(f"Reached Waypoint {self.current_wp_idx + 1}")
                self.current_wp_idx += 1
                
                # Check if we just reached the middle waypoint (WP 2)
                if self.current_wp_idx == 3:
                    self.transition_to("HOLD_MID")

        elif self.state == "HOLD_MID":
            # Hold at the middle waypoint (WP 2)
            target_x, target_y = self.waypoints[2]
            self.publish_trajectory_setpoint(target_x, target_y, self.flight_altitude)
            
            if self.state_timer == 0:
                self.current_tracker = DriftTracker(target_x, target_y)
                self.drift_trackers["HOLD_MID"] = self.current_tracker
                
            self.state_timer += 1
            if self.state_timer >= 40: # Hold for 4 seconds in the middle
                self.current_tracker = None
                self.transition_to("NAVIGATE")

        elif self.state == "HOLD_END":
            target_x, target_y = self.waypoints[-1]
            self.publish_trajectory_setpoint(target_x, target_y, self.flight_altitude)
            
            if self.state_timer == 0:
                self.current_tracker = DriftTracker(target_x, target_y)
                self.drift_trackers["HOLD_END"] = self.current_tracker
                
            self.state_timer += 1
            if self.state_timer >= 40: # Hold for 4 seconds before landing
                self.current_tracker = None
                self.transition_to("LAND")

        elif self.state == "LAND":
            if self.state_timer == 0:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            self.state_timer += 1
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.transition_to("DONE")

        elif self.state == "DONE":
            self.get_logger().info("=== Phase 2.5 Mission Complete ===")
            if "HOLD_MID" in self.drift_trackers:
                self.get_logger().info(f"Mid Hold Drift: {self.drift_trackers['HOLD_MID'].generate_report()}")
            if "HOLD_END" in self.drift_trackers:
                self.get_logger().info(f"End Hold Drift: {self.drift_trackers['HOLD_END'].generate_report()}")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PositionHoldNode()
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

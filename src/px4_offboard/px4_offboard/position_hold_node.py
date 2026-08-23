#!/usr/bin/env python3
"""
position_hold_node.py — Phase 2.5 GPS-Denied Position Hold Mission

Executes a rigid state machine to validate GPS-denied position hold performance.
Reads EKF2 readiness, arms cleanly, performs a hover-move-hover-return sequence,
and logs comprehensive drift metrics.
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
            # Only track if Z is relatively stable (we're not currently climbing/descending heavily)
            if self.state in ["HOLD", "HOLD_NORTH"]:
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
        msg.yaw = 0.0
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

        if self.state == "WARMUP":
            self.publish_trajectory_setpoint(4.5, 0.0, -2.0)
            if self.ekf2_ready:
                self.transition_to("ARM")
            else:
                self.state_timer += 1
                if self.state_timer >= self.warmup_timeout:
                    self.get_logger().error("EKF2 alignment timed out. Aborting mission.")
                    self.transition_to("DONE")

        elif self.state == "ARM":
            self.publish_trajectory_setpoint(4.5, 0.0, -2.0)
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if self.state_timer % 10 == 0:
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            elif self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                if self.state_timer % 10 == 0:
                    attempts = self.state_timer // 10
                    if attempts > 3:
                        self.get_logger().error("Failed to arm cleanly after 3 attempts. Aborting.")
                        self.transition_to("DONE")
                    else:
                        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
            else:
                self.transition_to("TAKEOFF")
            self.state_timer += 1

        elif self.state == "TAKEOFF":
            self.publish_trajectory_setpoint(4.5, 0.0, -2.0)
            # Check if altitude is within 0.15m of target (-2.0)
            if abs(self.current_z - (-2.0)) < 0.15:
                if self.state_timer > 20: # Must be stable for 2 seconds
                    self.current_tracker = DriftTracker(4.5, 0.0)
                    self.drift_trackers["HOLD_1"] = self.current_tracker
                    self.transition_to("HOLD")
                self.state_timer += 1
            else:
                self.state_timer = 0

        elif self.state == "HOLD":
            self.publish_trajectory_setpoint(4.5, 0.0, -2.0)
            self.state_timer += 1
            if self.state_timer >= 300: # 30 seconds
                self.current_tracker = None
                self.transition_to("MOVE_NORTH")

        elif self.state == "MOVE_NORTH":
            self.publish_trajectory_setpoint(6.0, 0.0, -2.0)
            self.state_timer += 1
            dist = math.sqrt((self.current_x - 6.0)**2 + (self.current_y - 0.0)**2)
            if dist < 0.3 or self.state_timer >= 150:
                self.current_tracker = DriftTracker(6.0, 0.0)
                self.drift_trackers["HOLD_NORTH"] = self.current_tracker
                self.transition_to("HOLD_NORTH")

        elif self.state == "HOLD_NORTH":
            self.publish_trajectory_setpoint(6.0, 0.0, -2.0)
            self.state_timer += 1
            if self.state_timer >= 150: # 15 seconds
                self.current_tracker = None
                self.transition_to("RETURN")

        elif self.state == "RETURN":
            self.publish_trajectory_setpoint(4.5, 0.0, -2.0)
            self.state_timer += 1
            dist = math.sqrt((self.current_x - 4.5)**2 + (self.current_y - 0.0)**2)
            if dist < 0.3 or self.state_timer >= 150:
                self.transition_to("LAND")

        elif self.state == "LAND":
            if self.state_timer == 0:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            
            self.state_timer += 1
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.transition_to("DONE")

        elif self.state == "DONE":
            self.get_logger().info("=== Phase 2.5 Mission Report ===")
            if "HOLD_1" in self.drift_trackers:
                self.get_logger().info(f"Initial 30s Hold Drift: {self.drift_trackers['HOLD_1'].generate_report()}")
            if "HOLD_NORTH" in self.drift_trackers:
                self.get_logger().info(f"North 15s Hold Drift: {self.drift_trackers['HOLD_NORTH'].generate_report()}")
            self.get_logger().info("================================")
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

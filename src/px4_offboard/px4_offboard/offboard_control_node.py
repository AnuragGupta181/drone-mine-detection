#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleCommandAck


class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control_node')

        # Configure Best-Effort QoS for PX4 microXRCE-DDS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers (Commands to PX4)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers (Telemetry from PX4)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self.status_callback, qos_profile)
        self.ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack_v1', self.ack_callback, qos_profile)

        # State Variables
        self.nav_state = 0
        self.arming_state = 0
        self.offboard_counter = 0
        self.armed_counter = 0
        self.arm_attempts = 0

        # Main 10Hz Loop (100ms interval)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Offboard Control Node Started. Waiting for setup...")

    def status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def ack_callback(self, msg):
        self.get_logger().info(f"Command Ack: Result {msg.result} for Command {msg.command}")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-2.0, yaw=-3.14):
        # Coordinates in NED frame: z = -2.0 means 2 meters ABOVE takeoff altitude
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [x, y, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw
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

    def timer_callback(self):
        # Human is at X=2.5m, Y=0.0m in Start Zone.
        # Hover target: X=4.5m (2 meters in front of human towards minefield), Y=0.0m, Z=-2.0m (2m altitude)
        target_x = 4.5
        target_y = 0.0
        target_z = -2.0

        # 1. Always stream Heartbeat & Trajectory Setpoints continuously (> 2Hz required)
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(x=target_x, y=target_y, z=target_z, yaw=0.0)

        # 2. Warm-up phase: stream setpoints for first 30 cycles (3 sec) for PX4 EKF2 readiness
        if self.offboard_counter < 30:
            self.offboard_counter += 1
            return

        # 3. State Machine based on telemetry feedback
        if getattr(self, 'is_landing', False):
            if self.arming_state == VehicleStatus.ARMING_STATE_DISARMED and self.armed_counter > 200:
                self.get_logger().info("Autonomous Flight & Landing Complete! Mission successful.")
                self.timer.cancel()
            return

        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.offboard_counter % 10 == 0:
                self.get_logger().info("Requesting OFFBOARD mode...")
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        elif self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            if self.offboard_counter % 10 == 0:
                self.arm_attempts += 1
                if self.arm_attempts <= 3:
                    self.get_logger().info(f"Requesting ARM (attempt {self.arm_attempts})...")
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
                else:
                    self.get_logger().info(f"Requesting Force ARM for SITL (attempt {self.arm_attempts})...")
                    self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21966.0)

        else:
            # Armed and in OFFBOARD mode!
            if self.armed_counter == 0:
                self.get_logger().info(f"Vehicle ARMED and in OFFBOARD mode! Moving to (X={target_x}m, Y={target_y}m) in front of human at 2m altitude...")

            self.armed_counter += 1

            if self.armed_counter == 200:  # 20 seconds hover
                self.get_logger().info("20s hover in front of human complete. Initiating Autonomous Landing...")
                self.is_landing = True
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        self.offboard_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
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

#!/usr/bin/env python3
"""
sim_tf_publisher.py — Phase 2.3 Temporary Simulation TF Bridge Node

ARCHITECTURE & COMPLIANCE NOTE:
--------------------------------
- Purpose: Converts PX4 telemetry odometry into a standard ROS 2 TF transform (odom -> base_link)
  required by `slam_toolbox` to perform 2D scan matching.
- Temporary Prerequisite: This is a TEMPORARY simulation bridge for Phase 2.3 SLAM initialization
  and is NOT the final GPS-denied localization solution.
- Strict Ground-Truth Isolation: This node consumes PX4 telemetry (`/fmu/out/vehicle_odometry`) ONLY.
  It NEVER subscribes to or consumes Gazebo ground truth or `/ground_truth/*` topics.

DESIGN PATTERN (SOLID):
------------------------
- Single Responsibility Principle (SRP): Handles exclusively PX4 odometry frame validation,
  NED/FRD to ENU/FLU coordinate frame conversion, and TF broadcasting.
- Strategy Pattern: Frame conversions are encapsulated in dedicated `IFrameConverter` implementations.
"""

from abc import ABC, abstractmethod
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class IFrameConverter(ABC):
    """Abstract Strategy Interface for PX4 to ROS 2 Coordinate Frame Transformations."""

    @abstractmethod
    def convert_position(self, px4_pos):
        """Convert PX4 position array to ROS 2 ENU position [x, y, z]."""
        pass

    @abstractmethod
    def convert_orientation(self, px4_q):
        """Convert PX4 orientation quaternion to ROS 2 FLU quaternion [w, x, y, z]."""
        pass


class PX4NEDToROSENUConverter(IFrameConverter):
    """
    Converts PX4 NED (North-East-Down) local position and FRD (Forward-Right-Down) body orientation
    to ROS 2 ENU (East-North-Up) world frame and FLU (Forward-Left-Up) body frame.
    """

    def convert_position(self, px4_pos):
        # PX4 NED: x=North, y=East, z=Down
        # ROS ENU: x=East, y=North, z=Up
        x_enu = float(px4_pos[1])   # East
        y_enu = float(px4_pos[0])   # North
        z_enu = -float(px4_pos[2])  # -Down
        return x_enu, y_enu, z_enu

    def convert_orientation(self, px4_q):
        # PX4 Quaternion convention: q = [w, x, y, z] in FRD/NED
        # Convert FRD to FLU attitude quaternion:
        # q_enu = [w, x, -y, -z] for 180 deg yaw offset / axis swap
        w = float(px4_q[0])
        x = float(px4_q[1])
        y = float(px4_q[2])
        z = float(px4_q[3])

        # Quaternion rotation from NED to ENU frame (yaw +90 deg, pitch/roll swapped):
        # ENU_x = NED_y, ENU_y = NED_x, ENU_z = -NED_z
        # Using standard PX4-to-ROS ENU/FLU quaternion transform:
        q_w = 0.7071068 * w - 0.7071068 * z
        q_x = 0.7071068 * x + 0.7071068 * y
        q_y = 0.7071068 * y - 0.7071068 * x
        q_z = 0.7071068 * z + 0.7071068 * w

        # Normalize quaternion to prevent numerical drift
        norm = math.sqrt(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z)
        if norm > 1e-6:
            return q_w / norm, q_x / norm, q_y / norm, q_z / norm
        return 1.0, 0.0, 0.0, 0.0


class SimTFPublisherNode(Node):
    """
    ROS 2 Node consuming simulated PX4 VehicleOdometry and publishing the odom -> base_link TF.
    """

    # PX4 VehicleOdometry pose_frame constants
    POSE_FRAME_NED = 1
    POSE_FRAME_FRD = 2

    def __init__(self):
        super().__init__('sim_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.converter = PX4NEDToROSENUConverter()
        self.warned_unsupported_frames = set()

        # Best-effort QoS profile for PX4 DDS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        self.get_logger().info(
            "SimTFPublisher initialized. Publishing odom -> base_link TF from simulated PX4 odometry."
        )

    def odom_callback(self, msg: VehicleOdometry):
        # 1. Validate pose_frame field
        frame_id_val = msg.pose_frame
        if frame_id_val not in (self.POSE_FRAME_NED, self.POSE_FRAME_FRD):
            if frame_id_val not in self.warned_unsupported_frames:
                self.get_logger().warning(
                    f"Unsupported or uninitialized VehicleOdometry.pose_frame: {frame_id_val}. Skipping TF broadcast."
                )
                self.warned_unsupported_frames.add(frame_id_val)
            return

        # 2. Perform frame conversions
        x_enu, y_enu, z_enu = self.converter.convert_position(msg.position)
        q_w, q_x, q_y, q_z = self.converter.convert_orientation(msg.q)

        # 3. Construct TransformStamped (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu

        # ROS geometry_msgs/Quaternion uses (x, y, z, w) order
        t.transform.rotation.x = q_x
        t.transform.rotation.y = q_y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w

        # 4. Broadcast transform to ROS 2 TF tree
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimTFPublisherNode()
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

#!/usr/bin/env python3
"""
sim_tf_publisher.py — Phase 2.3/2.5 Simulation Odometry TF Bridge

Publishes the `odom -> base_link` TF transform for `slam_toolbox`.
Supports Gazebo model odometry (`/model/x500_lidar_2d_0/odometry` or `/model/x500_lidar_2d/odometry`)
to provide a stable, non-spinning odometric reference frame for SLAM scan matching.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


class SimTFPublisherNode(Node):
    def __init__(self):
        super().__init__('sim_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.known_scan_frames = set(['base_link', 'lidar_link'])

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 1. Gazebo Model Odometry (Instance 0 and generic topics)
        self.gz_odom_sub1 = self.create_subscription(
            Odometry,
            '/model/x500_lidar_2d_0/odometry',
            self.gz_odom_callback,
            10
        )
        self.gz_odom_sub2 = self.create_subscription(
            Odometry,
            '/model/x500_lidar_2d/odometry',
            self.gz_odom_callback,
            10
        )

        # 2. Subscribe to /scan to automatically bridge any frame_id from Gazebo to base_link
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 3. Fallback subscriber: PX4 VehicleOdometry
        self.px4_odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.px4_odom_callback,
            qos_profile
        )

        self.has_gz_odom = False
        self.get_logger().info("SimTFPublisher initialized with dynamic scan frame TF bridge.")

    def scan_callback(self, msg: LaserScan):
        frame = msg.header.frame_id
        if frame and frame not in self.known_scan_frames:
            self.known_scan_frames.add(frame)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = frame
            t.transform.translation.x = 0.12
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.26
            t.transform.rotation.w = 1.0
            self.static_broadcaster.sendTransform(t)
            self.get_logger().info(f"Auto-bridged scan frame '{frame}' -> 'base_link' TF.")

    def gz_odom_callback(self, msg: Odometry):
        self.has_gz_odom = True

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Gazebo ENU Position
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

    def px4_odom_callback(self, msg: VehicleOdometry):
        if self.has_gz_odom:
            return

        if msg.pose_frame not in (1, 2):
            return

        x_enu = float(msg.position[1])
        y_enu = float(msg.position[0])
        z_enu = -float(msg.position[2])

        w, x, y, z = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])
        q_w = 0.7071068 * w - 0.7071068 * z
        q_x = 0.7071068 * x + 0.7071068 * y
        q_y = 0.7071068 * y - 0.7071068 * x
        q_z = 0.7071068 * z + 0.7071068 * w

        norm = math.sqrt(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z)
        if norm > 1e-6:
            q_w, q_x, q_y, q_z = q_w / norm, q_x / norm, q_y / norm, q_z / norm

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x_enu
        t.transform.translation.y = y_enu
        t.transform.translation.z = z_enu

        t.transform.rotation.x = q_x
        t.transform.rotation.y = q_y
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w

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

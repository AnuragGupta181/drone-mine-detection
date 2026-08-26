#!/usr/bin/env python3
"""
sim_tf_publisher.py — Phase 2.3/2.5 Simulation Odometry TF Bridge

Publishes the `odom -> base_link` TF transform for `slam_toolbox`.
Uses Gazebo model odometry (bridged via ros_gz_bridge) to keep TF stamps
aligned with the /clock (sim time) so slam_toolbox doesn't reject scans.

Key fixes vs previous version:
  - Uses msg.header.stamp (sim time) for TF, not wall clock.
  - Publishes a start-up identity odom->base_link at t=0 so slam_toolbox
    doesn't fail before the first Gazebo odometry packet arrives.
  - No VehicleOdometry subscription (avoids px4_msgs deserialization crash).
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class SimTFPublisherNode(Node):
    def __init__(self):
        super().__init__('sim_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # ── Publish identity odom→base_link immediately at startup ───────────
        # This prevents slam_toolbox from rejecting early scans because the
        # TF tree is incomplete before the first Gazebo odometry packet.
        self._publish_identity_odom()

        # ── Gazebo odometry subscriptions (both naming conventions) ──────────
        self.create_subscription(
            Odometry,
            '/model/x500_lidar_2d_0/odometry',
            self.gz_odom_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/model/x500_lidar_2d/odometry',
            self.gz_odom_callback,
            10
        )

        self.get_logger().info('SimTFPublisher ready — waiting for Gazebo odometry.')

    def _publish_identity_odom(self):
        """Publish an identity odom->base_link at the node start time."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation.w = 1.0   # identity quaternion
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published startup identity odom->base_link TF.')

    def gz_odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # Use the message header stamp (sim time) so slam_toolbox clock matches
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

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

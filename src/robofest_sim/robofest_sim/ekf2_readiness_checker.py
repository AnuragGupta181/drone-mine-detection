#!/usr/bin/env python3
"""
ekf2_readiness_checker.py — Phase 2.5 EKF2 Readiness Pre-Flight Gate

This read-only ROS 2 node monitors PX4's EstimatorStatusFlags to ensure EKF2
is fully aligned and configured for External Vision (SLAM) GPS-denied flight.
It acts as a safety gate, publishing a boolean readiness state.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from px4_msgs.msg import EstimatorStatusFlags

class EKF2ReadinessChecker(Node):
    def __init__(self):
        super().__init__('ekf2_readiness_checker')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            EstimatorStatusFlags,
            '/fmu/out/estimator_status_flags',
            self.status_callback,
            qos_profile
        )

        self.ready_pub = self.create_publisher(Bool, '/ekf2_ready', 10)

        self.is_ready = False
        self.last_flags = None

        # Log status every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("EKF2 Readiness Checker Started. Waiting for EstimatorStatusFlags...")

    def status_callback(self, msg):
        self.last_flags = msg

        # Evaluate Readiness Criteria
        tilt_ok = msg.cs_tilt_align
        yaw_ok = msg.cs_yaw_align
        ev_pos_ok = msg.cs_ev_pos
        ev_yaw_ok = msg.cs_ev_yaw
        gps_off = not msg.cs_gnss_pos
        no_hdg_fault = not msg.fs_bad_hdg
        no_fake_pos = not msg.cs_fake_pos

        self.is_ready = (tilt_ok and yaw_ok and ev_pos_ok and ev_yaw_ok and 
                         gps_off and no_hdg_fault and no_fake_pos)

        # Publish the state
        ready_msg = Bool()
        ready_msg.data = self.is_ready
        self.ready_pub.publish(ready_msg)

    def timer_callback(self):
        if self.last_flags is None:
            self.get_logger().info("Waiting for EKF2 telemetry...")
            return

        if self.is_ready:
            self.get_logger().info("EKF2 is READY for GPS-Denied Flight.", once=True)
        else:
            self.get_logger().warn(
                f"EKF2 NOT READY: "
                f"tilt_align={self.last_flags.cs_tilt_align}, "
                f"yaw_align={self.last_flags.cs_yaw_align}, "
                f"ev_pos={self.last_flags.cs_ev_pos}, "
                f"ev_yaw={self.last_flags.cs_ev_yaw}, "
                f"gnss_pos={self.last_flags.cs_gnss_pos} (must be False), "
                f"bad_hdg_fault={self.last_flags.fs_bad_hdg} (must be False), "
                f"fake_pos={self.last_flags.cs_fake_pos} (must be False)"
            )

def main(args=None):
    rclpy.init(args=args)
    node = EKF2ReadinessChecker()
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

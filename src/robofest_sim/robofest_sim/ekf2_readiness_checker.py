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

        # Readiness: tilt + yaw aligned, position from GPS or EV, no faults
        tilt_ok = msg.cs_tilt_align
        yaw_ok = msg.cs_yaw_align
        has_pos = msg.cs_gnss_pos or msg.cs_ev_pos   # either GPS or SLAM
        no_hdg_fault = not msg.fs_bad_hdg
        no_fake_pos = not msg.cs_fake_pos

        self.is_ready = (tilt_ok and yaw_ok and has_pos and no_hdg_fault and no_fake_pos)

        # Publish the state
        ready_msg = Bool()
        ready_msg.data = self.is_ready
        self.ready_pub.publish(ready_msg)

    def timer_callback(self):
        if self.last_flags is None:
            self.get_logger().info("Waiting for EKF2 telemetry from PX4...")
            return

        if self.is_ready:
            self.get_logger().info("EKF2 is READY for flight.", once=True)
        else:
            f = self.last_flags
            self.get_logger().warn(
                f"EKF2 NOT READY: "
                f"tilt_align={f.cs_tilt_align}, "
                f"yaw_align={f.cs_yaw_align}, "
                f"gnss_pos={f.cs_gnss_pos}, "
                f"ev_pos={f.cs_ev_pos}, "
                f"bad_hdg={f.fs_bad_hdg}, "
                f"fake_pos={f.cs_fake_pos}"
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

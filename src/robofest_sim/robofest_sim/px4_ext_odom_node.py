#!/usr/bin/env python3
"""
px4_ext_odom_node.py — Phase 2.4 PX4 External Odometry & EKF2 Interface

This node bridges SLAM-derived poses into PX4's EKF2 state estimator by publishing
VehicleOdometry messages to /fmu/in/vehicle_visual_odometry.

It explicitly queries the TF tree for the `map -> base_link` transform, applies mathematically
rigorous ENU->NED and FLU->FRD transformations, and formats timestamps in microseconds
as required by PX4's uORB architecture.

Note: Quality is set to 100 as a placeholder, velocity fields are explicitly NaN, 
and EKF2_EV_NOISE_MD semantics are documented.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped

class PX4ExtOdomNode(Node):
    def __init__(self):
        super().__init__('px4_ext_odom_node')

        # Best-effort QoS profile for PX4 DDS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile
        )

        # TF Listener for map -> base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State tracking
        self.reset_counter = 0
        self.last_tf_stamp = None

        # Transformation Matrices
        # ENU (East, North, Up) to NED (North, East, Down)
        self.R_ENU_to_NED = np.array([
            [0, 1,  0],
            [1, 0,  0],
            [0, 0, -1]
        ])

        # FLU (Forward, Left, Up) to FRD (Forward, Right, Down)
        # Note: R_FLU_to_FRD is its own inverse/transpose.
        self.R_FLU_to_FRD = np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ])

        # Timer to run at ~50 Hz
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("PX4 External Odometry Node initialized. Waiting for map -> base_link TF...")
        
        # Log NED alignment requirement warning
        self.get_logger().warn(
            "NED FRAME ALIGNMENT REQUIREMENT: Publishing POSE_FRAME_NED is only valid if the SLAM 'map' "
            "frame is aligned with the PX4 local NED reference. slam_toolbox does not align this automatically. "
            "Ensure initial heading alignment or EKF2 positions will be corrupted."
        )

    def quaternion_to_rotation_matrix(self, q):
        """Converts a quaternion [x, y, z, w] to a 3x3 rotation matrix."""
        x, y, z, w = q
        R = np.array([
            [1 - 2*(y**2 + z**2),     2*(x*y - z*w),         2*(x*z + y*w)],
            [2*(x*y + z*w),           1 - 2*(x**2 + z**2),   2*(y*z - x*w)],
            [2*(x*z - y*w),           2*(y*z + x*w),         1 - 2*(x**2 + y**2)]
        ])
        return R

    def rotation_matrix_to_quaternion(self, R):
        """Converts a 3x3 rotation matrix back to a quaternion [x, y, z, w] and returns [w, x, y, z].
        Note: The returned format is PX4's expected [w, x, y, z].
        """
        tr = np.trace(R)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
            
        # Normalize
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
            
        return w, x, y, z

    def timer_callback(self):
        try:
            # Lookup latest transform
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
        except TransformException as ex:
            # If TF fails, just return quietly (normal on startup)
            return

        # Check for TF discontinuities/jumps to increment reset_counter
        # A simple check: if time jumped backwards (e.g. simulation reset)
        current_tf_stamp = t.header.stamp
        if self.last_tf_stamp is not None:
            if (current_tf_stamp.sec < self.last_tf_stamp.sec):
                self.reset_counter = (self.reset_counter + 1) % 256
        self.last_tf_stamp = current_tf_stamp

        msg = VehicleOdometry()
        
        # 1. Timestamps (MUST BE IN MICROSECONDS)
        now_ns = self.get_clock().now().nanoseconds
        msg.timestamp = int(now_ns / 1000)
        
        tf_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
        if tf_ns > 0:
            msg.timestamp_sample = int(tf_ns / 1000)
        else:
            msg.timestamp_sample = msg.timestamp

        # 2. Frames
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN

        # 3. Position (ENU -> NED)
        p_enu = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])
        p_ned = self.R_ENU_to_NED @ p_enu
        
        msg.position = [float(p_ned[0]), float(p_ned[1]), float(p_ned[2])]

        # 4. Orientation (FLU -> FRD via matrix composition)
        q_enu = [
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ]
        
        # R_ENU_FLU is the rotation matrix derived from the TF quaternion
        R_ENU_FLU = self.quaternion_to_rotation_matrix(q_enu)
        
        # R_NED_FRD = R_ENU_to_NED * R_ENU_FLU * R_FLU_to_FRD
        R_NED_FRD = self.R_ENU_to_NED @ R_ENU_FLU @ self.R_FLU_to_FRD
        
        # Convert back to quaternion [w, x, y, z] for PX4
        msg.q = [float(v) for v in self.rotation_matrix_to_quaternion(R_NED_FRD)]

        # 5. Velocity
        msg.velocity = [float('nan'), float('nan'), float('nan')]

        # 6. Variances
        # Note: Simulation assumption. These will act as lower bounds if EKF2_EV_NOISE_MD=0
        # or will be overridden by parameters if EKF2_EV_NOISE_MD=1.
        msg.position_variance = [0.01, 0.01, 0.01]
        msg.orientation_variance = [0.01, 0.01, 0.01]
        msg.velocity_variance = [float('nan'), float('nan'), float('nan')]

        # 7. Metadata
        msg.reset_counter = self.reset_counter
        msg.quality = 100  # Placeholder, PX4 currently ignores this

        # Publish
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PX4ExtOdomNode()
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

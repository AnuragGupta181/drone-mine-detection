#!/usr/bin/env python3

import os
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32

class MissionEvaluator(Node):
    def __init__(self):
        super().__init__('mission_evaluator')
        
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('time_limit_sec', 600.0)
        self.declare_parameter('min_clearance_m', 1.0)
        
        manifest_path = self.get_parameter('manifest_path').get_parameter_value().string_value
        if not manifest_path:
            manifest_path = '/home/ubuntu/px4_ros2_ws/src/robofest_sim/worlds/generated/stage1_manifest.json'
            
        self.time_limit = self.get_parameter('time_limit_sec').get_parameter_value().double_value
        self.min_clearance = self.get_parameter('min_clearance_m').get_parameter_value().double_value
        
        self.manifest = self.load_manifest(manifest_path)
        self.start_time = self.get_clock().now()
        self.safety_violations = 0
        
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.timer_pub = self.create_publisher(Float32, '/mission/elapsed_time', 10)
        
        self.drone_pose_sub = self.create_subscription(
            PoseStamped, 
            '/ground_truth/drone_pose', 
            self.drone_pose_callback, 
            10
        )
        
        self.eval_timer = self.create_timer(1.0, self.eval_loop)
        self.get_logger().info("MissionEvaluator initialized. Competition limit: 600s, Min clearance: 1.0m.")

    def load_manifest(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"Manifest path not found: {path}")
            return None
        with open(path, 'r') as f:
            return json.load(f)

    def drone_pose_callback(self, msg: PoseStamped):
        if not self.manifest:
            return
        dx = msg.pose.position.x
        dy = msg.pose.position.y
        
        mines = self.manifest.get('mines', [])
        for m in mines:
            mx, my = m['position'][0], m['position'][1]
            dist = math.hypot(dx - mx, dy - my)
            if dist < self.min_clearance:
                self.safety_violations += 1
                self.get_logger().warn(
                    f"SAFETY VIOLATION #{self.safety_violations}: Drone at ({dx:.2f}, {dy:.2f}) "
                    f"is {dist:.2f}m from Mine {m['id']} (min clearance: {self.min_clearance}m)!"
                )

    def eval_loop(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        timer_msg = Float32()
        timer_msg.data = float(elapsed)
        self.timer_pub.publish(timer_msg)
        
        status_msg = String()
        if elapsed > self.time_limit:
            status_msg.data = f"MISSION TIMEOUT ({elapsed:.1f}s > {self.time_limit}s)"
        else:
            status_msg.data = f"MISSION ACTIVE: Elapsed={elapsed:.1f}s, Safety Violations={self.safety_violations}"
            
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionEvaluator()
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

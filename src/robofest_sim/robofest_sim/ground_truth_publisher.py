#!/usr/bin/env python3

import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_rate', 10.0)
        
        manifest_path = self.get_parameter('manifest_path').get_parameter_value().string_value
        if not manifest_path:
            manifest_path = '/home/ubuntu/px4_ros2_ws/src/robofest_sim/worlds/generated/stage1_manifest.json'
            
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        self.get_logger().info(f"Loading ground truth manifest from: {manifest_path}")
        self.manifest_data = self.load_manifest(manifest_path)
        
        # Publishers
        self.mines_pub = self.create_publisher(MarkerArray, '/ground_truth/mines', 10)
        self.obstacles_pub = self.create_publisher(MarkerArray, '/ground_truth/obstacles', 10)
        self.start_zone_pub = self.create_publisher(Marker, '/ground_truth/start_zone', 10)
        self.exit_zone_pub = self.create_publisher(Marker, '/ground_truth/exit_zone', 10)
        self.human_pub = self.create_publisher(PoseStamped, '/ground_truth/human_pose', 10)
        self.drone_pub = self.create_publisher(Marker, '/ground_truth/drone_marker', 10)

        # Track live drone position from Gazebo odometry
        self.drone_pose = None
        self.odom_sub1 = self.create_subscription(
            Odometry, '/model/x500_lidar_2d_0/odometry', self.odom_callback, 10)
        self.odom_sub2 = self.create_subscription(
            Odometry, '/model/x500_lidar_2d/odometry', self.odom_callback, 10)
        
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.get_logger().info("GroundTruthPublisher initialized and publishing at 10 Hz.")

    def odom_callback(self, msg: Odometry):
        """Track drone position from Gazebo ground-truth odometry."""
        self.drone_pose = msg.pose.pose

    def load_manifest(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"Manifest file does not exist: {path}")
            return None
        with open(path, 'r') as f:
            return json.load(f)

    def timer_callback(self):
        if not self.manifest_data:
            return
            
        now = self.get_clock().now().to_msg()
        header = Header(stamp=now, frame_id=self.frame_id)
        
        # 1. Publish Simulated Mines Markers (Traffic Cones)
        mine_array = MarkerArray()
        for idx, m in enumerate(self.manifest_data.get('mines', [])):
            marker = Marker()
            marker.header = header
            marker.ns = "ground_truth_mines"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(m['position'][0])
            marker.pose.position.y = float(m['position'][1])
            marker.pose.position.z = float(m['position'][2]) + 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 0.9
            mine_array.markers.append(marker)
        self.mines_pub.publish(mine_array)
        
        # 2. Publish Static Obstacles Markers
        obs_array = MarkerArray()
        for idx, o in enumerate(self.manifest_data.get('obstacles', [])):
            marker = Marker()
            marker.header = header
            marker.ns = "ground_truth_obstacles"
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(o['position'][0])
            marker.pose.position.y = float(o['position'][1])
            marker.pose.position.z = float(o['position'][2]) + 0.75
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(o['size'][0])
            marker.scale.y = float(o['size'][1])
            marker.scale.z = float(o['size'][2])
            marker.color.r = 0.6
            marker.color.g = 0.4
            marker.color.b = 0.2
            marker.color.a = 0.8
            obs_array.markers.append(marker)
        self.obstacles_pub.publish(obs_array)

        # 3. Publish Start Zone Marker (Green Box)
        sz = self.manifest_data.get('zones', {}).get('start_zone', {})
        if sz:
            sz_marker = Marker()
            sz_marker.header = header
            sz_marker.ns = "start_zone"
            sz_marker.id = 0
            sz_marker.type = Marker.CUBE
            sz_marker.action = Marker.ADD
            sz_marker.pose.position.x = (sz['x_min'] + sz['x_max']) / 2.0
            sz_marker.pose.position.y = (sz['y_min'] + sz['y_max']) / 2.0
            sz_marker.pose.position.z = 0.01
            sz_marker.pose.orientation.w = 1.0
            sz_marker.scale.x = float(sz['x_max'] - sz['x_min'])
            sz_marker.scale.y = float(sz['y_max'] - sz['y_min'])
            sz_marker.scale.z = 0.02
            sz_marker.color.r = 0.1
            sz_marker.color.g = 0.8
            sz_marker.color.b = 0.1
            sz_marker.color.a = 0.4
            self.start_zone_pub.publish(sz_marker)

        # 4. Publish Exit Zone Marker (Blue Box)
        ez = self.manifest_data.get('zones', {}).get('exit_zone', {})
        if ez:
            ez_marker = Marker()
            ez_marker.header = header
            ez_marker.ns = "exit_zone"
            ez_marker.id = 0
            ez_marker.type = Marker.CUBE
            ez_marker.action = Marker.ADD
            ez_marker.pose.position.x = (ez['x_min'] + ez['x_max']) / 2.0
            ez_marker.pose.position.y = (ez['y_min'] + ez['y_max']) / 2.0
            ez_marker.pose.position.z = 0.01
            ez_marker.pose.orientation.w = 1.0
            ez_marker.scale.x = float(ez['x_max'] - ez['x_min'])
            ez_marker.scale.y = float(ez['y_max'] - ez['y_min'])
            ez_marker.scale.z = 0.02
            ez_marker.color.r = 0.1
            ez_marker.color.g = 0.3
            ez_marker.color.b = 0.9
            ez_marker.color.a = 0.4
            self.exit_zone_pub.publish(ez_marker)

        # 5. Publish Static Human Pose
        human_cfg = self.manifest_data.get('human', {})
        if human_cfg:
            h_pose = PoseStamped()
            h_pose.header = header
            pos = human_cfg.get('position', [2.5, 0.0, 0.0])
            h_pose.pose.position.x = float(pos[0])
            h_pose.pose.position.y = float(pos[1])
            h_pose.pose.position.z = float(pos[2])
            h_pose.pose.orientation.w = 1.0
            self.human_pub.publish(h_pose)

        # 6. Publish Live Drone Marker (tracks actual Gazebo position)
        drone_marker = Marker()
        drone_marker.header = header
        drone_marker.ns = "drone_marker"
        drone_marker.id = 0
        drone_marker.type = Marker.CYLINDER
        drone_marker.action = Marker.ADD
        if self.drone_pose is not None:
            drone_marker.pose = self.drone_pose
        else:
            # Default: spawn position
            drone_marker.pose.position.x = 4.5
            drone_marker.pose.position.y = 0.0
            drone_marker.pose.position.z = 0.2
            drone_marker.pose.orientation.w = 1.0
        drone_marker.scale.x = 0.8
        drone_marker.scale.y = 0.8
        drone_marker.scale.z = 0.15
        drone_marker.color.r = 0.9
        drone_marker.color.g = 0.7
        drone_marker.color.b = 0.1
        drone_marker.color.a = 0.95
        self.drone_pub.publish(drone_marker)

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
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

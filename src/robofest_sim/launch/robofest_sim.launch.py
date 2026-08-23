import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_robofest_sim = FindPackageShare('robofest_sim')
    
    default_config_path = PathJoinSubstitution([pkg_robofest_sim, 'config', 'stage1.yaml'])
    default_rviz_path = PathJoinSubstitution([pkg_robofest_sim, 'rviz', 'robofest_sim.rviz'])
    default_manifest_path = PathJoinSubstitution([pkg_robofest_sim, 'worlds', 'generated', 'stage1_manifest.json'])
    default_world_path = PathJoinSubstitution([pkg_robofest_sim, 'worlds', 'generated', 'stage1_seeded.sdf'])
    
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to Stage 1 YAML configuration'
    )
    
    seed_arg = DeclareLaunchArgument(
        'seed',
        default_value='42',
        description='Random seed for scenario generator'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    publish_lidar_tf_arg = DeclareLaunchArgument(
        'publish_lidar_static_tf',
        default_value='false',
        description='Whether to publish static base_link -> lidar_link TF if not provided by Gazebo'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to launch slam_toolbox 2D SLAM'
    )

    ext_odom_arg = DeclareLaunchArgument(
        'ext_odom',
        default_value='true',
        description='Whether to launch px4_ext_odom_node for Phase 2.4'
    )

    # 1. Scenario Generator Node Action
    generate_scenario_cmd = Node(
        package='robofest_sim',
        executable='scenario_generator',
        name='scenario_generator',
        output='screen',
        arguments=['--config', LaunchConfiguration('config_file'), '--seed', LaunchConfiguration('seed')]
    )

    # 2. Ground Truth Publisher Node
    ground_truth_node = Node(
        package='robofest_sim',
        executable='ground_truth_publisher',
        name='ground_truth_publisher',
        output='screen',
        parameters=[{
            'manifest_path': default_manifest_path,
            'frame_id': 'world',
            'publish_rate': 10.0
        }]
    )

    # 3. Mission Evaluator Node
    mission_evaluator_node = Node(
        package='robofest_sim',
        executable='mission_evaluator',
        name='mission_evaluator',
        output='screen',
        parameters=[{
            'manifest_path': default_manifest_path,
            'time_limit_sec': 600.0,
            'min_clearance_m': 1.0
        }]
    )

    # 4. Sensor Health & Diagnostic Monitor Node (Phase 2.2)
    sensor_health_node = Node(
        package='robofest_sim',
        executable='sensor_health_monitor',
        name='sensor_health_monitor',
        output='screen'
    )

    # 5. Temporary Simulation TF Publisher (Phase 2.3: odom -> base_link)
    sim_tf_node = Node(
        package='robofest_sim',
        executable='sim_tf_publisher',
        name='sim_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. 2D LiDAR SLAM Toolbox Node (Phase 2.3)
    slam_params_path = PathJoinSubstitution([pkg_robofest_sim, 'config', 'slam_toolbox_params.yaml'])
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_path]
    )

    # 7. Static World -> Map TF Publisher (Syncs RViz ground truth markers in world frame with map frame)
    static_map_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_world_tf',
        output='screen',
        arguments=['4.5', '0', '0', '0', '0', '0', 'world', 'map']
    )

    # 8. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_path],
        parameters=[{'use_sim_time': True}]
    )

    # 9. ROS-Gazebo Bridge (LiDAR Scan)
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/x500_lidar_2d/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/x500_lidar_2d_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ]
    )

    # 10. Static TF: base_link -> lidar_link (x500_lidar_2d offset: z=0.26, x=0.12)
    lidar_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=['0.12', '0', '0.26', '0', '0', '0', 'base_link', 'lidar_link']
    )

    # 11. PX4 External Odometry Node (Phase 2.4)
    px4_ext_odom_node = Node(
        package='robofest_sim',
        executable='px4_ext_odom_node',
        name='px4_ext_odom_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 12. EKF2 Readiness Checker (Phase 2.5)
    ekf2_readiness_checker_node = Node(
        package='robofest_sim',
        executable='ekf2_readiness_checker',
        name='ekf2_readiness_checker',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        config_arg,
        seed_arg,
        rviz_arg,
        ext_odom_arg,
        publish_lidar_tf_arg,
        slam_arg,
        generate_scenario_cmd,
        ground_truth_node,
        mission_evaluator_node,
        sensor_health_node,
        sim_tf_node,
        slam_toolbox_node,
        static_map_world_tf,
        rviz_node,
        ros_gz_bridge_node,
        lidar_static_tf_node,
        px4_ext_odom_node,
        ekf2_readiness_checker_node
    ])
    
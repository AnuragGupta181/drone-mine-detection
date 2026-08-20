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

    # 5. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_path]
    )

    return LaunchDescription([
        config_arg,
        seed_arg,
        rviz_arg,
        generate_scenario_cmd,
        ground_truth_node,
        mission_evaluator_node,
        sensor_health_node,
        rviz_node
    ])

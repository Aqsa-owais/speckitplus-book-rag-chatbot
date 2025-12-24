#!/usr/bin/env python3
"""
Launch file for simple perception pipeline example
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Perception pipeline node
    perception_pipeline_node = Node(
        package='perception_pipeline_examples',
        executable='simple_perception_pipeline',
        name='simple_perception_pipeline',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/camera/image_raw', '/camera/image_rect_color'),
            ('/camera/camera_info', '/camera/camera_info')
        ],
        output='screen'
    )

    # RViz node for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('perception_pipeline_examples'),
        'rviz',
        'perception_pipeline.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # TF2 static transform publisher for camera
    camera_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(camera_tf_publisher)
    ld.add_action(perception_pipeline_node)

    # Add RViz with event handler to start after other nodes
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=perception_pipeline_node,
            on_start=[rviz_node]
        )
    ))

    return ld
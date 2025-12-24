#!/usr/bin/env python3
"""
Launch file for simple navigation example
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('navigation_example'),
            'maps',
            'simple_map.yaml'
        ]),
        description='Full path to map file to load')

    # Include Nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )

    # Simple navigation node
    simple_navigation_node = Node(
        package='navigation_example',
        executable='simple_navigation',
        name='simple_navigation',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/cmd_vel', '/cmd_vel'),
            ('/navigate_to_pose', '/navigate_to_pose')
        ],
        output='screen'
    )

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}
        ],
        output='screen'
    )

    # Local and global costmap parameters
    local_costmap_params = PathJoinSubstitution([
        FindPackageShare('navigation_example'),
        'config',
        'local_costmap_params.yaml'
    ])

    global_costmap_params = PathJoinSubstitution([
        FindPackageShare('navigation_example'),
        'config',
        'global_costmap_params.yaml'
    ])

    # Lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': 'true'},
            {'node_names': ['map_server', 'local_costmap', 'global_costmap']}
        ]
    )

    # RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('navigation_example'),
        'rviz',
        'navigation_example.rviz'
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

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_file_cmd)

    # Add nodes
    ld.add_action(lifecycle_manager)
    ld.add_action(map_server_node)
    ld.add_action(nav2_launch)
    ld.add_action(simple_navigation_node)

    # Add RViz with event handler to start after other nodes
    ld.add_action(RegisterEventHandler(
        OnProcessStart(
            target_action=simple_navigation_node,
            on_start=[rviz_node]
        )
    ))

    return ld
---
sidebar_position: 9
---

# Navigation and Path Planning using Nav2

## Introduction

Navigation is a critical capability for autonomous robots, enabling them to move safely and efficiently from one location to another in complex environments. The Navigation2 (Nav2) stack is the state-of-the-art navigation framework for ROS 2, providing a complete solution for robot navigation including path planning, path following, obstacle avoidance, and recovery behaviors.

This guide covers the fundamentals of navigation and path planning using Nav2, including system architecture, configuration, and integration with perception systems.

## Navigation Architecture

### Nav2 System Overview

Nav2 follows a behavior tree architecture that allows for flexible and robust navigation behaviors:

```text
Nav2 System
├── Navigation Server
│   ├── Lifecycle Manager
│   ├── Behavior Tree Engine
│   └── Action Servers
├── Planner Server
│   ├── Global Planner (A*, Dijkstra, etc.)
│   └── Local Planner (DWA, TEB, etc.)
├── Controller Server
│   ├── Path Following Controller
│   └── Velocity Smoothing
├── Recovery Server
│   ├── Spin Recovery
│   ├── Backup Recovery
│   └── Wait Recovery
└── Map Server
    ├── Static Map Server
    └── Costmap Server
```

### Key Components

#### 1. Global Planner

The global planner computes a path from start to goal in the static map:

```cpp
// Example C++ code for custom global planner plugin
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class CustomGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
        std::shared_ptr<nav2_costmap_2d::Costmap2D> global_costmap,
        std::string local_footprint_str) override
    {
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
    }

    void cleanup() override {}

    void activate() override {}

    void deactivate() override {}

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override
    {
        nav_msgs::msg::Path path;

        // Implement path planning algorithm (e.g., A* or Dijkstra)
        if (computePath(start, goal, path)) {
            return path;
        } else {
            // Return empty path if no path found
            return nav_msgs::msg::Path{};
        }
    }

private:
    bool computePath(const geometry_msgs::msg::PoseStamped& start,
                     const geometry_msgs::msg::PoseStamped& goal,
                     nav_msgs::msg::Path& path)
    {
        // Implementation of path planning algorithm
        // This is a simplified example
        path.poses.resize(10);  // Example path with 10 points

        for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = global_frame_;
            pose.pose.position.x = start.pose.position.x +
                                  (goal.pose.position.x - start.pose.position.x) * i / 10.0;
            pose.pose.position.y = start.pose.position.y +
                                  (goal.pose.position.y - start.pose.position.y) * i / 10.0;
            pose.pose.orientation.w = 1.0;  // No rotation
            path.poses[i] = pose;
        }

        return true;  // Path found
    }

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::string global_frame_;
    nav2_costmap_2d::Costmap2D* costmap_;
};
```

#### 2. Local Planner

The local planner follows the global path while avoiding obstacles:

```python
# Example Python code for local planning (conceptual)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class LocalPlanner(Node):
    def __init__(self):
        super().__init__('local_planner')

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.global_plan_sub = self.create_subscription(
            Path, '/global_plan', self.global_plan_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current state
        self.current_pose = None
        self.current_twist = None
        self.laser_data = None
        self.global_plan = None
        self.current_plan_idx = 0

        # Local planner parameters
        self.lookahead_dist = 0.5
        self.max_vel = 0.5
        self.max_ang_vel = 1.0
        self.safety_dist = 0.3

    def odom_callback(self, msg):
        """Update current pose and twist from odometry"""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def laser_callback(self, msg):
        """Update laser scan data"""
        self.laser_data = msg

    def global_plan_callback(self, msg):
        """Update global plan"""
        self.global_plan = msg.poses
        self.current_plan_idx = 0

    def compute_velocity_commands(self):
        """Compute velocity commands based on global plan and local obstacles"""
        if not self.global_plan or not self.laser_data or not self.current_pose:
            return Twist()

        # Get next point on global plan
        target_point = self.get_next_waypoint()
        if not target_point:
            return Twist()  # Stop if no target

        # Calculate desired direction
        dx = target_point.pose.position.x - self.current_pose.position.x
        dy = target_point.pose.position.y - self.current_pose.position.y
        dist_to_target = np.sqrt(dx*dx + dy*dy)

        # Calculate desired angle
        desired_angle = np.arctan2(dy, dx)

        # Get current robot angle
        current_angle = self.quaternion_to_yaw(self.current_pose.orientation)

        # Calculate angle error
        angle_error = self.normalize_angle(desired_angle - current_angle)

        # Create velocity command
        cmd_vel = Twist()

        # Check for obstacles
        if self.has_obstacles_ahead():
            # Emergency stop or obstacle avoidance
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.avoid_obstacles()
        else:
            # Move toward target
            cmd_vel.linear.x = min(self.max_vel * dist_to_target, self.max_vel)
            cmd_vel.angular.z = self.max_ang_vel * angle_error

        return cmd_vel

    def get_next_waypoint(self):
        """Get next waypoint on global plan"""
        if not self.global_plan or self.current_plan_idx >= len(self.global_plan):
            return None

        # Find next waypoint within lookahead distance
        for i in range(self.current_plan_idx, len(self.global_plan)):
            wp = self.global_plan[i]
            dx = wp.pose.position.x - self.current_pose.position.x
            dy = wp.pose.position.y - self.current_pose.position.y
            dist = np.sqrt(dx*dx + dy*dy)

            if dist >= self.lookahead_dist:
                self.current_plan_idx = i
                return wp

        # Return last waypoint if reached end of plan
        return self.global_plan[-1] if self.global_plan else None

    def has_obstacles_ahead(self):
        """Check if there are obstacles in front of robot"""
        if not self.laser_data:
            return False

        # Check laser readings in front of robot
        front_idx = len(self.laser_data.ranges) // 2
        scan_width = len(self.laser_data.ranges) // 8  # Check ~45 degrees in front

        for i in range(front_idx - scan_width//2, front_idx + scan_width//2):
            if 0 < self.laser_data.ranges[i] < self.safety_dist:
                return True

        return False

    def avoid_obstacles(self):
        """Simple obstacle avoidance behavior"""
        if not self.laser_data:
            return 0.0

        # Simple strategy: turn away from closest obstacle
        min_dist = float('inf')
        min_idx = len(self.laser_data.ranges) // 2

        for i, dist in enumerate(self.laser_data.ranges):
            if 0 < dist < min_dist:
                min_dist = dist
                min_idx = i

        # Turn away from obstacle
        center_idx = len(self.laser_data.ranges) // 2
        if min_idx < center_idx:
            # Obstacle on the left, turn right
            return -self.max_ang_vel
        else:
            # Obstacle on the right, turn left
            return self.max_ang_vel

    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
```

## Nav2 Configuration

### Basic Nav2 Configuration File

```yaml
# Navigation configuration file: nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: /opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
    default_nav_to_pose_bt_xml: /opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_compute_path_through_poses_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_have_reached_goal_condition_bt_node
    - nav2_are_paths_valid_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_truncate_path_action_bt_node
    - nav2_goal_updater_node_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_path_expiring_timer_condition
    - nav2_distance_traveled_condition_bt_node
    - nav2_single_trigger_bt_node
    - nav2_is_battery_low_condition_bt_node
    - nav2_navigate_through_poses_action_bt_node
    - nav2_navigate_to_pose_action_bt_node

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # DWB parameters
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "dwb_core::DWBLocalPlanner"
      smooth_rotate: True

      # DWB Core parameters
      dwb_core:
        plugin: "dwb_core::DWBLocalPlanner"
        debug_trajectory_details: True
        min_vel_x: 0.0
        min_vel_y: 0.0
        max_vel_x: 0.5
        max_vel_y: 0.0
        max_vel_theta: 1.0
        min_speed_xy: 0.0
        max_speed_xy: 0.5
        min_speed_theta: 0.0
        acc_lim_x: 2.5
        acc_lim_y: 0.0
        acc_lim_theta: 3.2
        decel_lim_x: -2.5
        decel_lim_y: 0.0
        decel_lim_theta: -3.2
        vx_samples: 20
        vy_samples: 5
        vtheta_samples: 20
        sim_time: 1.7
        linear_granularity: 0.05
        angular_granularity: 0.1
        transform_tolerance: 0.2
        xy_goal_tolerance: 0.25
        trans_stopped_velocity: 0.25
        short_circuit_trajectory_evaluation: True
        stateful: True
        critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
        BaseObstacle.scale: 0.02
        PathAlign.scale: 32.0
        PathAlign.forward_point_distance: 0.1
        GoalAlign.scale: 24.0
        GoalAlign.forward_point_distance: 0.1
        PathDist.scale: 32.0
        GoalDist.scale: 24.0
        RotateToGoal.scale: 32.0
        RotateToGoal.slowing_factor: 5.0
        RotateToGoal.lookahead_time: -1.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: false
      width: 100
      height: 100
      resolution: 0.05
      origin_x: -50.0
      origin_y: -50.0
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: False
      rolling_window: True
      width: 3
      height: 3
      resolution: 0.05
      origin_x: -1.5
      origin_y: -1.5
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
      sim_granularity: 0.017
      angle: 1.57
    backup:
      plugin: "nav2_recoveries/BackUp"
      sim_granularity: 0.0044
      duration: 10.0
      backup_vel: -0.025
    wait:
      plugin: "nav2_recoveries/Wait"
      sim_time: 5.0

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
```

### Nav2 Launch File

```python
# Navigation launch file: navigation_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_navigation'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_bt_navigator'),
            'behavior_trees',
            'navigate_w_replanning_and_recovery.xml'
        ]),
        description='Full path to the behavior tree xml file to use')

    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='true',
        description='Whether to set the map subscriber QoS to transient local')

    # Navigation Server
    start_nav2_node = Node(
        package='nav2_navigation_server',
        executable='navigation_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    # Planner Server
    start_planner_server = Node(
        package='nav2_planner_server',
        executable='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    # Controller Server
    start_controller_server = Node(
        package='nav2_controller_server',
        executable='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    # BT Navigator
    start_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[params_file, {
            'default_bt_xml_filename': default_bt_xml_filename}],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    # Lifecycle Manager
    start_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                   {'autostart': autostart},
                   {'node_names': ['navigation_server',
                                  'planner_server',
                                  'controller_server',
                                  'bt_navigator']}]
    )

    # Return launch description
    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_map_subscribe_transient_local_cmd,
        start_nav2_node,
        start_planner_server,
        start_controller_server,
        start_bt_navigator,
        start_lifecycle_manager
    ])
```

## Path Planning Algorithms

### A* Path Planning Implementation

```python
import heapq
import numpy as np

class AStarPlanner:
    def __init__(self, costmap, resolution=0.05):
        self.costmap = costmap
        self.resolution = resolution
        self.height, self.width = costmap.shape

    def plan_path(self, start, goal):
        """
        Plan path using A* algorithm

        Args:
            start: tuple (x, y) in meters
            goal: tuple (x, y) in meters

        Returns:
            path: list of (x, y) tuples in meters
        """
        # Convert world coordinates to grid coordinates
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)

        # Check if start and goal are valid
        if not self.is_valid_cell(start_grid) or not self.is_valid_cell(goal_grid):
            return []

        # Initialize open and closed sets
        open_set = [(0, start_grid)]  # (f_score, (x, y))
        heapq.heapify(open_set)

        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_grid:
                return self.reconstruct_path(came_from, current)

            # Check neighbors
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal_grid)

                    # Add to open set if not already there
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def heuristic(self, pos1, pos2):
        """Calculate heuristic (Manhattan distance)"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def distance(self, pos1, pos2):
        """Calculate distance between two positions"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return np.sqrt(dx*dx + dy*dy)

    def get_neighbors(self, pos):
        """Get valid neighboring cells"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue  # Skip current cell

                neighbor = (pos[0] + dx, pos[1] + dy)

                if self.is_valid_cell(neighbor) and self.is_traversable(neighbor):
                    neighbors.append(neighbor)

        return neighbors

    def is_valid_cell(self, pos):
        """Check if position is within bounds"""
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def is_traversable(self, pos):
        """Check if cell is traversable (cost < 253, which is lethal obstacle in costmap)"""
        x, y = pos
        return self.costmap[y, x] < 253

    def world_to_grid(self, world_pos):
        """Convert world coordinates to grid coordinates"""
        x_world, y_world = world_pos
        x_grid = int((x_world + self.width * self.resolution / 2) / self.resolution)
        y_grid = int((y_world + self.height * self.resolution / 2) / self.resolution)
        return (x_grid, y_grid)

    def grid_to_world(self, grid_pos):
        """Convert grid coordinates to world coordinates"""
        x_grid, y_grid = grid_pos
        x_world = x_grid * self.resolution - self.width * self.resolution / 2
        y_world = y_grid * self.resolution - self.height * self.resolution / 2
        return (x_world, y_world)

    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)

        # Convert grid coordinates back to world coordinates
        world_path = [self.grid_to_world(pos) for pos in reversed(path)]
        return world_path
```

### Dijkstra's Algorithm Implementation

```python
import heapq
import numpy as np

class DijkstraPlanner:
    def __init__(self, costmap, resolution=0.05):
        self.costmap = costmap
        self.resolution = resolution
        self.height, self.width = costmap.shape

    def plan_path(self, start, goal):
        """
        Plan path using Dijkstra's algorithm

        Args:
            start: tuple (x, y) in meters
            goal: tuple (x, y) in meters

        Returns:
            path: list of (x, y) tuples in meters
        """
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)

        if not self.is_valid_cell(start_grid) or not self.is_valid_cell(goal_grid):
            return []

        # Initialize distances and previous nodes
        distances = {}
        previous = {}
        visited = set()

        # Priority queue: (distance, (x, y))
        pq = [(0, start_grid)]
        distances[start_grid] = 0

        while pq:
            current_dist, current = heapq.heappop(pq)

            if current in visited:
                continue

            visited.add(current)

            if current == goal_grid:
                break

            for neighbor in self.get_neighbors(current):
                if neighbor in visited:
                    continue

                edge_weight = self.get_edge_weight(current, neighbor)
                new_distance = distances[current] + edge_weight

                if neighbor not in distances or new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_distance, neighbor))

        # Reconstruct path
        if goal_grid not in previous:
            return []  # No path found

        path = self.reconstruct_path(previous, start_grid, goal_grid)
        return path

    def get_edge_weight(self, pos1, pos2):
        """Calculate edge weight based on costmap values"""
        x1, y1 = pos1
        x2, y2 = pos2

        # Get cost of destination cell
        cost = self.costmap[y2, x2]

        # Normalize cost (costmap values are typically 0-254)
        normalized_cost = cost / 254.0

        # Calculate geometric distance
        dx = x2 - x1
        dy = y2 - y1
        geometric_distance = np.sqrt(dx*dx + dy*dy)

        # Combined weight
        weight = geometric_distance * (1 + normalized_cost)
        return weight

    def get_neighbors(self, pos):
        """Get valid neighboring cells"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                neighbor = (pos[0] + dx, pos[1] + dy)

                if self.is_valid_cell(neighbor) and self.is_traversable(neighbor):
                    neighbors.append(neighbor)

        return neighbors

    def is_valid_cell(self, pos):
        """Check if position is within bounds"""
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height

    def is_traversable(self, pos):
        """Check if cell is traversable"""
        x, y = pos
        return self.costmap[y, x] < 253

    def world_to_grid(self, world_pos):
        """Convert world coordinates to grid coordinates"""
        x_world, y_world = world_pos
        x_grid = int((x_world + self.width * self.resolution / 2) / self.resolution)
        y_grid = int((y_world + self.height * self.resolution / 2) / self.resolution)
        return (x_grid, y_grid)

    def grid_to_world(self, grid_pos):
        """Convert grid coordinates to world coordinates"""
        x_grid, y_grid = grid_pos
        x_world = x_grid * self.resolution - self.width * self.resolution / 2
        y_world = y_grid * self.resolution - self.height * self.resolution / 2
        return (x_world, y_world)

    def reconstruct_path(self, previous, start, goal):
        """Reconstruct path from previous dictionary"""
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = previous[current]
        path.append(start)

        # Convert to world coordinates
        world_path = [self.grid_to_world(pos) for pos in reversed(path)]
        return world_path
```

## Behavior Trees in Navigation

### Custom Behavior Tree Nodes

```python
import py_trees
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient

class NavigateToPoseBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(NavigateToPoseBehavior, self).__init__(name)
        self.node = node
        self.navigate_to_pose_client = ActionClient(
            self.node, NavigateToPose, 'navigate_to_pose'
        )
        self.goal_sent = False
        self.goal_pose = None

    def initialise(self):
        """Initialise the behavior tree node"""
        self.node.get_logger().info(f"Sending navigation goal: {self.goal_pose}")

        # Wait for action server
        if not self.navigate_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.feedback_message = "NavigateToPose action server not available"
            return py_trees.common.Status.FAILURE

        # Send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose
        self.navigate_to_pose_client.send_goal_async(goal_msg)
        self.goal_sent = True

    def update(self):
        """Update the behavior tree node"""
        if not self.goal_sent:
            return py_trees.common.Status.RUNNING

        # Check if goal is still active
        # In a real implementation, you would check the actual goal status
        # This is a simplified version
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Terminate the behavior tree node"""
        self.node.get_logger().info(f"Terminated with status: {new_status}")

def create_simple_navigation_tree(node):
    """Create a simple navigation behavior tree"""

    # Root
    root = py_trees.composites.Sequence("NavigationSequence")

    # Create navigation behavior
    nav_behavior = NavigateToPoseBehavior("NavigateToPose", node)

    # Add to root
    root.add_child(nav_behavior)

    return root
```

## Navigation with Isaac ROS Integration

### Isaac ROS Perception Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np

class IsaacPerceptionNavIntegration(Node):
    def __init__(self):
        super().__init__('isaac_perception_nav_integration')

        # Subscribe to perception outputs
        self.semantic_seg_sub = self.create_subscription(
            Image,
            '/segmentation_mask',
            self.semantic_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/depth',
            self.depth_callback,
            10
        )

        self.object_detection_sub = self.create_subscription(
            Image,
            '/detections',
            self.detection_callback,
            10
        )

        # Publish enhanced costmaps
        self.enhanced_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/enhanced_costmap',
            10
        )

        # Subscribe to navigation status
        self.nav_status_sub = self.create_subscription(
            Bool,
            '/navigation_active',
            self.nav_status_callback,
            10
        )

        # Navigation goal publisher
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        # Perception data storage
        self.semantic_map = None
        self.depth_map = None
        self.detections = None
        self.navigation_active = False

        # Integration parameters
        self.dynamic_obstacle_threshold = 0.5  # Confidence threshold
        self.traversable_classes = ['road', 'sidewalk', 'grass']
        self.obstacle_classes = ['car', 'person', 'tree', 'building']

    def semantic_callback(self, msg):
        """Process semantic segmentation data"""
        # Convert ROS image to numpy array
        # This is simplified - in practice would use cv_bridge
        semantic_data = np.frombuffer(msg.data, dtype=np.uint8)
        semantic_map = semantic_data.reshape((msg.height, msg.width))

        self.semantic_map = semantic_map
        self.update_costmap_from_semantics()

    def depth_callback(self, msg):
        """Process depth data"""
        # Convert depth image to numpy array
        depth_data = np.frombuffer(msg.data, dtype=np.float32)
        depth_map = depth_data.reshape((msg.height, msg.width))

        self.depth_map = depth_map
        self.update_costmap_from_depth()

    def detection_callback(self, msg):
        """Process object detection data"""
        # Process object detection results
        # This would parse detection messages
        self.detections = msg
        self.update_costmap_from_detections()

    def nav_status_callback(self, msg):
        """Update navigation status"""
        self.navigation_active = msg.data

    def update_costmap_from_semantics(self):
        """Update costmap based on semantic segmentation"""
        if self.semantic_map is None or not self.navigation_active:
            return

        # Update costmap based on semantic classes
        height, width = self.semantic_map.shape
        costmap = np.zeros((height, width), dtype=np.uint8)

        for row in range(height):
            for col in range(width):
                semantic_class = self.semantic_map[row, col]

                if semantic_class in self.obstacle_classes:
                    costmap[row, col] = 254  # Lethal obstacle
                elif semantic_class in self.traversable_classes:
                    costmap[row, col] = 0    # Free space
                else:
                    costmap[row, col] = 128  # Unknown

        # Publish enhanced costmap
        self.publish_enhanced_costmap(costmap)

    def update_costmap_from_depth(self):
        """Update costmap based on depth information"""
        if self.depth_map is None or not self.navigation_active:
            return

        # Update costmap based on depth thresholds
        # Mark areas as obstacles if depth suggests proximity to objects
        height, width = self.depth_map.shape
        costmap = np.zeros((height, width), dtype=np.uint8)

        for row in range(height):
            for col in range(width):
                depth = self.depth_map[row, col]

                if depth < 0.5:  # Close to obstacle
                    costmap[row, col] = 254
                elif depth < 1.0:  # Near obstacle
                    costmap[row, col] = 200
                elif depth > 10.0:  # Far away
                    costmap[row, col] = 0

        # Blend with semantic costmap
        self.publish_enhanced_costmap(costmap)

    def update_costmap_from_detections(self):
        """Update costmap based on object detections"""
        if self.detections is None or not self.navigation_active:
            return

        # Update costmap based on detected objects
        # Mark detection bounding boxes as obstacles
        # This is a simplified implementation
        pass

    def publish_enhanced_costmap(self, costmap_data):
        """Publish enhanced costmap combining multiple perception sources"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Set map metadata
        msg.info.resolution = 0.05  # meters per cell
        msg.info.width = costmap_data.shape[1]
        msg.info.height = costmap_data.shape[0]
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten costmap data
        msg.data = costmap_data.flatten().tolist()

        self.enhanced_costmap_pub.publish(msg)
```

## Navigation Recovery Behaviors

### Custom Recovery Behaviors

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class NavigationRecovery(Node):
    def __init__(self):
        super().__init__('navigation_recovery')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.recovery_status_pub = self.create_publisher(Bool, '/recovery_active', 10)

        # Recovery behaviors
        self.active_recovery = None
        self.recovery_timer = None

        # Parameters
        self.backup_speed = -0.1
        self.backup_duration = 2.0
        self.spin_angle = 1.57  # 90 degrees
        self.wait_duration = 5.0

    def execute_spin_recovery(self):
        """Execute spin recovery behavior"""
        self.get_logger().info("Executing spin recovery")
        self.active_recovery = "spin"

        # Create timer for recovery duration
        self.recovery_timer = self.create_timer(
            0.1, self.spin_recovery_callback
        )

        self.start_recovery_timer(self.spin_angle / 0.5)  # Assuming 0.5 rad/s spin speed

    def spin_recovery_callback(self):
        """Spin recovery callback"""
        if self.active_recovery != "spin":
            return

        cmd_vel = Twist()
        cmd_vel.angular.z = 0.5  # Spin at 0.5 rad/s
        self.cmd_vel_pub.publish(cmd_vel)

    def execute_backup_recovery(self):
        """Execute backup recovery behavior"""
        self.get_logger().info("Executing backup recovery")
        self.active_recovery = "backup"

        # Create timer for recovery duration
        self.recovery_timer = self.create_timer(
            0.1, self.backup_recovery_callback
        )

        self.start_recovery_timer(self.backup_duration)

    def backup_recovery_callback(self):
        """Backup recovery callback"""
        if self.active_recovery != "backup":
            return

        cmd_vel = Twist()
        cmd_vel.linear.x = self.backup_speed
        self.cmd_vel_pub.publish(cmd_vel)

    def execute_wait_recovery(self):
        """Execute wait recovery behavior"""
        self.get_logger().info("Executing wait recovery")
        self.active_recovery = "wait"

        # Stop robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        # Create timer for wait duration
        self.recovery_timer = self.create_timer(
            self.wait_duration, self.wait_recovery_complete
        )

    def wait_recovery_complete(self):
        """Complete wait recovery"""
        self.get_logger().info("Wait recovery completed")
        self.cleanup_recovery()

    def start_recovery_timer(self, duration):
        """Start timer for recovery behavior"""
        self.recovery_start_time = self.get_clock().now()
        self.recovery_duration = duration

        # Create periodic timer to check if recovery should stop
        self.check_timer = self.create_timer(0.1, self.check_recovery_completion)

    def check_recovery_completion(self):
        """Check if recovery behavior should complete"""
        if self.active_recovery:
            elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9

            if elapsed >= self.recovery_duration:
                self.cleanup_recovery()

    def cleanup_recovery(self):
        """Clean up active recovery behavior"""
        if self.active_recovery:
            self.get_logger().info(f"Cleaning up {self.active_recovery} recovery")

            # Stop robot
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)

            # Cancel timers
            if self.recovery_timer:
                self.recovery_timer.cancel()
            if self.check_timer:
                self.check_timer.cancel()

            # Reset active recovery
            self.active_recovery = None

            # Publish recovery status
            status_msg = Bool()
            status_msg.data = False
            self.recovery_status_pub.publish(status_msg)
```

## Performance Optimization

### Multi-Level Path Planning

```python
class MultiLevelPathPlanner:
    def __init__(self, global_costmap, local_costmap):
        self.global_costmap = global_costmap
        self.local_costmap = local_costmap

        # Hierarchical planners
        self.coarse_planner = AStarPlanner(global_costmap, resolution=0.5)  # Coarse grid
        self.fine_planner = AStarPlanner(local_costmap, resolution=0.05)   # Fine grid

    def plan_path_multilevel(self, start, goal):
        """
        Plan path using multilevel approach:
        1. Coarse planning for global path
        2. Fine planning for local obstacle avoidance
        """
        # 1. Plan coarse path at global level
        coarse_path = self.coarse_planner.plan_path(start, goal)

        if not coarse_path:
            return []

        # 2. Refine path using fine resolution
        refined_path = []

        for i in range(len(coarse_path) - 1):
            segment_start = coarse_path[i]
            segment_end = coarse_path[i + 1]

            # Plan fine-grained path between coarse waypoints
            fine_segment = self.fine_planner.plan_path(segment_start, segment_end)

            if fine_segment:
                refined_path.extend(fine_segment[:-1])  # Exclude last point to avoid duplication
            else:
                # If fine planning fails, use the coarse path
                refined_path.append(segment_start)

        # Add final goal point
        if coarse_path:
            refined_path.append(coarse_path[-1])

        return refined_path
```

## Troubleshooting Common Issues

### 1. Navigation Failures
- **Problem**: Robot fails to reach goal
- **Solutions**:
  - Check costmap inflation settings
  - Verify localization accuracy
  - Adjust path planning parameters
  - Check for sensor issues

### 2. Oscillation in Path Following
- **Problem**: Robot oscillates around path
- **Solutions**:
  - Increase lookahead distance
  - Adjust PID controller gains
  - Smooth path with spline interpolation
  - Check velocity limits

### 3. Local Minima in Navigation
- **Problem**: Robot gets stuck in local minima
- **Solutions**:
  - Implement better obstacle avoidance
  - Use more sophisticated local planners
  - Add recovery behaviors
  - Improve costmap resolution

### 4. Performance Issues
- **Problem**: Navigation system too slow
- **Solutions**:
  - Reduce costmap resolution
  - Optimize path planning algorithms
  - Use multi-threading
  - Implement path caching

## Integration with Isaac ROS

### Launch Configuration for Isaac ROS Navigation

```xml
<!-- Isaac ROS Navigation Launch File -->
<launch>
  <!-- Map server -->
  <node pkg="nav2_map_server" exec="map_server" name="map_server">
    <param name="yaml_filename" value="$(find-pkg-share my_maps)/maps/my_map.yaml"/>
  </node>

  <!-- AMCL localization -->
  <node pkg="nav2_amcl" exec="amcl" name="amcl">
    <param name="set_initial_pose" value="true"/>
    <param name="initial_pose.x" value="0.0"/>
    <param name="initial_pose.y" value="0.0"/>
    <param name="initial_pose.z" value="0.0"/>
    <param name="initial_pose.yaw" value="0.0"/>
  </node>

  <!-- Isaac ROS perception nodes -->
  <node pkg="isaac_ros_visual_slam" exec="isaac_ros_visual_slam" name="visual_slam">
    <param name="enable_occupancy_map_generation" value="true"/>
  </node>

  <!-- Isaac perception to Nav2 integration -->
  <node pkg="my_perception_integration" exec="perception_nav_integration" name="perception_nav_integration"/>

  <!-- Navigation stack -->
  <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
    <arg name="use_sim_time" value="false"/>
    <arg name="params_file" value="$(find-pkg-share my_robot_navigation)/config/nav2_params.yaml"/>
  </include>

  <!-- RViz for visualization -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share nav2_bringup)/rviz/nav2_default_view.rviz"/>
</launch>
```

## Summary

Navigation and path planning using Nav2 provides a comprehensive solution for autonomous robot navigation. The system combines global path planning with local obstacle avoidance, recovery behaviors, and sophisticated costmap management. By integrating with Isaac ROS perception systems, robots can achieve robust navigation in complex environments. Proper configuration and tuning of the various parameters are essential for optimal performance. The multilevel approach and behavior tree architecture provide flexibility and robustness for real-world navigation challenges.
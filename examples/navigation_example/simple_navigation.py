#!/usr/bin/env python3
"""
Simple Isaac ROS Navigation Example

This example demonstrates basic navigation using Nav2 and Isaac ROS integration.
The example includes:
1. Waypoint following
2. Obstacle avoidance
3. Basic path planning
4. Integration with Isaac ROS perception
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Duration as BuiltinDuration
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import numpy as np
import math


class SimpleNavigation(Node):
    """
    A simple navigation node that demonstrates:
    1. Waypoint following using Nav2
    2. Obstacle avoidance
    3. Basic navigation safety
    """

    def __init__(self):
        super().__init__('simple_navigation')

        # Action client for Nav2 navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/navigation_status', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Navigation state
        self.current_pose = None
        self.navigation_active = False
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.obstacle_detected = False
        self.min_obstacle_distance = 0.5  # meters

        # Navigation parameters
        self.linear_speed = 0.3  # m/s
        self.angular_speed = 0.5  # rad/s
        self.arrival_threshold = 0.5  # meters

        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_control)

        self.get_logger().info("Simple navigation node initialized")

    def odom_callback(self, msg):
        """Update current robot pose from odometry"""
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Process laser scan data for obstacle detection"""
        # Check for obstacles in front of the robot
        front_scan_start = len(msg.ranges) // 2 - 30  # Look at front ±30°
        front_scan_end = len(msg.ranges) // 2 + 30

        min_distance = float('inf')
        for i in range(front_scan_start, front_scan_end):
            if 0 < msg.ranges[i] < min_distance:
                min_distance = msg.ranges[i]

        self.obstacle_detected = min_distance < self.min_obstacle_distance

        if self.obstacle_detected:
            self.get_logger().warn(f"Obstacle detected at {min_distance:.2f}m")

    def set_waypoints(self, waypoints):
        """Set navigation waypoints"""
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.get_logger().info(f"Set {len(waypoints)} waypoints")

    def navigate_to_waypoints(self):
        """Start navigating to waypoints"""
        if not self.waypoints:
            self.get_logger().warn("No waypoints set")
            return

        self.get_logger().info("Starting navigation to waypoints")
        self.navigation_active = True
        self.navigate_to_next_waypoint()

    def navigate_to_next_waypoint(self):
        """Navigate to the next waypoint in the list"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info("Reached final waypoint")
            self.navigation_active = False
            self.publish_status(False)
            return

        goal_pose = self.waypoints[self.current_waypoint_idx]
        self.send_navigation_goal(goal_pose)

    def send_navigation_goal(self, pose):
        """Send navigation goal to Nav2"""
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Navigation action server not available")
            return

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree_id = ""  # Use default behavior tree

        # Send goal
        self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback):
        """Handle navigation feedback"""
        # This is called periodically during navigation
        pass

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Successfully reached waypoint")
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx < len(self.waypoints):
                # Navigate to next waypoint
                self.navigate_to_next_waypoint()
            else:
                # All waypoints reached
                self.get_logger().info("Completed all waypoints")
                self.navigation_active = False
                self.publish_status(False)
        else:
            self.get_logger().warn(f"Navigation failed with status: {result.status}")
            self.navigation_active = False
            self.publish_status(False)

    def navigation_control(self):
        """Main navigation control loop"""
        if not self.navigation_active:
            return

        if not self.current_pose:
            return

        if self.obstacle_detected:
            self.handle_obstacle()
        else:
            # Check if we need to navigate to next waypoint
            if self.current_waypoint_idx < len(self.waypoints):
                # Check if we're close to current waypoint
                current_wp = self.waypoints[self.current_waypoint_idx]
                distance = self.calculate_distance_to_pose(self.current_pose, current_wp.pose)

                if distance < self.arrival_threshold:
                    self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}")
                    self.current_waypoint_idx += 1
                    if self.current_waypoint_idx < len(self.waypoints):
                        self.navigate_to_next_waypoint()

    def calculate_distance_to_pose(self, pose1, pose2):
        """Calculate 2D distance between two poses"""
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx*dx + dy*dy)

    def handle_obstacle(self):
        """Handle obstacle detection"""
        self.get_logger().info("Obstacle detected, stopping robot")

        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def publish_status(self, active):
        """Publish navigation status"""
        status_msg = Bool()
        status_msg.data = active
        self.status_pub.publish(status_msg)

    def stop_navigation(self):
        """Stop current navigation"""
        self.navigation_active = False
        self.publish_status(False)

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        self.get_logger().info("Navigation stopped")


def create_waypoints():
    """Create example waypoints for navigation"""
    waypoints = []

    # Waypoint 1: Move forward 2 meters
    wp1 = PoseStamped()
    wp1.header.frame_id = 'map'
    wp1.pose.position.x = 2.0
    wp1.pose.position.y = 0.0
    wp1.pose.position.z = 0.0
    wp1.pose.orientation.w = 1.0  # No rotation
    waypoints.append(wp1)

    # Waypoint 2: Move to (2, 2)
    wp2 = PoseStamped()
    wp2.header.frame_id = 'map'
    wp2.pose.position.x = 2.0
    wp2.pose.position.y = 2.0
    wp2.pose.position.z = 0.0
    # Face towards next waypoint
    wp2.pose.orientation.w = 1.0
    waypoints.append(wp2)

    # Waypoint 3: Move to (0, 2)
    wp3 = PoseStamped()
    wp3.header.frame_id = 'map'
    wp3.pose.position.x = 0.0
    wp3.pose.position.y = 2.0
    wp3.pose.position.z = 0.0
    wp3.pose.orientation.w = 1.0
    waypoints.append(wp3)

    # Waypoint 4: Return to origin
    wp4 = PoseStamped()
    wp4.header.frame_id = 'map'
    wp4.pose.position.x = 0.0
    wp4.pose.position.y = 0.0
    wp4.pose.position.z = 0.0
    wp4.pose.orientation.w = 1.0
    waypoints.append(wp4)

    return waypoints


def main(args=None):
    rclpy.init(args=args)

    navigation_node = SimpleNavigation()

    # Create and set waypoints
    waypoints = create_waypoints()
    navigation_node.set_waypoints(waypoints)

    # Start navigation
    navigation_node.navigate_to_waypoints()

    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        navigation_node.get_logger().info("Navigation interrupted by user")
    finally:
        navigation_node.stop_navigation()
        navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
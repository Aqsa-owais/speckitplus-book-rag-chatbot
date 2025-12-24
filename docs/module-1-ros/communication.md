---
sidebar_position: 5
---

# Communication between AI Logic and Robot Controllers

## Introduction

One of the core challenges in robotics is enabling effective communication between high-level AI logic and low-level robot controllers. The AI logic makes decisions based on perception, planning, and reasoning, while controllers execute these decisions by commanding actuators and processing sensor feedback. ROS 2 provides the communication infrastructure to bridge these different system components.

## Architecture Overview

### High-Level vs. Low-Level Components

**High-Level AI Components:**
- Perception systems (object detection, SLAM, etc.)
- Planning systems (path planning, task planning)
- Reasoning systems (decision making, cognitive architectures)
- User interfaces and command systems

**Low-Level Controller Components:**
- Joint position/velocity/effort controllers
- Sensor drivers and processing
- Safety and emergency systems
- Hardware abstraction layers

### Communication Patterns

The communication between AI logic and robot controllers typically follows these patterns:
- **Commands**: AI logic sends commands to controllers
- **State feedback**: Controllers report robot state back to AI
- **Sensor data**: Controllers provide processed sensor data
- **Status updates**: Controllers report operational status

## Communication Channels

### Command Channels

AI logic sends commands to robot controllers through various message types:

#### Joint Commands
```python
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AICommander(Node):

    def __init__(self):
        super().__init__('ai_commander')
        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    def send_joint_command(self, joint_names, positions, velocities=None, accelerations=None):
        msg = JointTrajectory()
        msg.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        if velocities:
            point.velocities = velocities
        if accelerations:
            point.accelerations = accelerations

        # Set execution time (1 second from now)
        point.time_from_start.sec = 1
        msg.points = [point]

        self.joint_pub.publish(msg)
```

#### Velocity Commands
```python
from geometry_msgs.msg import Twist

class NavigationAI(Node):

    def __init__(self):
        super().__init__('navigation_ai')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_velocity_command(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
```

### State Feedback Channels

Controllers report robot state back to AI systems:

#### Joint State Feedback
```python
from sensor_msgs.msg import JointState

class StateMonitor(Node):

    def __init__(self):
        super().__init__('state_monitor')
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

    def joint_state_callback(self, msg):
        # Process joint positions, velocities, efforts
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 0.0
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
            effort = msg.effort[i] if i < len(msg.effort) else 0.0

            # Update AI system with current state
            self.update_robot_state(name, position, velocity, effort)
```

#### Robot State Feedback
```python
from nav_msgs.msg import Odometry

class OdomMonitor(Node):

    def __init__(self):
        super().__init__('odom_monitor')
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        # Extract position and orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation (quaternion)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Update AI navigation system
        self.update_navigation_state(x, y, z, qx, qy, qz, qw)
```

## Integration Patterns

### Behavior-Based Architecture

This pattern organizes robot behaviors as separate nodes that communicate through ROS topics:

```python
# Behavior 1: Obstacle Avoidance
class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg):
        # Process laser scan to detect obstacles
        min_distance = min(msg.ranges)

        if min_distance < 1.0:  # Too close to obstacle
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn away
            self.cmd_pub.publish(cmd)

# Behavior 2: Goal Navigation
class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')
        self.goal_sub = self.create_subscription(Pose, '/goal', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def goal_callback(self, msg):
        # Navigate to goal
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
```

### Action-Based Integration

For complex tasks that require feedback and cancelation:

```python
from rclpy.action import ActionServer, ActionClient
from control_msgs.action import FollowJointTrajectory

class AIPlanner(Node):

    def __init__(self):
        super().__init__('ai_planner')
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def execute_trajectory(self, joint_names, trajectory_points):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = trajectory_points

        self.trajectory_client.wait_for_server()
        send_goal_future = self.trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.trajectory_feedback_callback)

        send_goal_future.add_done_callback(self.trajectory_response_callback)

    def trajectory_feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Trajectory progress: {feedback_msg.feedback.actual.positions}')

    def trajectory_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Trajectory accepted, waiting for result...')
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.trajectory_result_callback)

    def trajectory_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Trajectory execution result: {result.error_code}')
```

## Safety Considerations

### Emergency Stop Integration

```python
from std_msgs.msg import Bool

class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.ai_cmd_sub = self.create_subscription(Twist, '/ai_cmd_vel', self.ai_cmd_callback, 10)
        self.controller_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def ai_cmd_callback(self, msg):
        # Check if command is safe
        if self.is_command_safe(msg):
            # Forward to controller
            self.controller_cmd_pub.publish(msg)
        else:
            # Send emergency stop
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)
            self.get_logger().error('Unsafe command detected, emergency stop activated!')

    def is_command_safe(self, cmd):
        # Implement safety checks
        if abs(cmd.linear.x) > 1.0:  # Max linear velocity
            return False
        if abs(cmd.angular.z) > 1.0:  # Max angular velocity
            return False
        return True
```

### State Validation

```python
class StateValidator(Node):

    def __init__(self):
        super().__init__('state_validator')
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.validate_state, 10)

    def validate_state(self, msg):
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 0.0

            # Check if position is within safe limits
            if not self.is_position_safe(name, position):
                self.get_logger().error(f'Unsafe position for joint {name}: {position}')
                # Trigger safety response
                self.trigger_safety_response(name, position)

    def is_position_safe(self, joint_name, position):
        # Define safe limits for each joint
        limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-1.57, 1.57),
            # Add more joint limits as needed
        }

        if joint_name in limits:
            min_limit, max_limit = limits[joint_name]
            return min_limit <= position <= max_limit
        return True  # Assume safe if not specified
```

## Advanced Integration: AI-Controller Coordination

### Coordination Node

```python
class AIControllerCoordinator(Node):

    def __init__(self):
        super().__init__('ai_controller_coordinator')

        # AI communication
        self.ai_command_sub = self.create_subscription(String, '/ai_commands', self.ai_command_callback, 10)
        self.ai_feedback_pub = self.create_publisher(String, '/ai_feedback', 10)

        # Controller communication
        self.controller_status_sub = self.create_subscription(String, '/controller_status', self.controller_status_callback, 10)
        self.controller_cmd_pub = self.create_publisher(String, '/controller_commands', 10)

        # State tracking
        self.current_state = "IDLE"
        self.ai_goals = []
        self.controller_status = {}

    def ai_command_callback(self, msg):
        command = msg.data
        if command.startswith("GOAL:"):
            goal = command[5:]  # Remove "GOAL:" prefix
            self.ai_goals.append(goal)
            self.execute_next_goal()
        elif command == "STOP":
            self.stop_robot()
        elif command == "RESET":
            self.reset_system()

    def controller_status_callback(self, msg):
        # Update controller status
        status = msg.data
        self.controller_status = self.parse_status(status)

        # Check if current goal is completed
        if self.current_state == "EXECUTING" and self.is_goal_completed():
            self.current_state = "IDLE"
            self.send_feedback("GOAL_COMPLETED")

    def execute_next_goal(self):
        if self.ai_goals and self.current_state == "IDLE":
            next_goal = self.ai_goals.pop(0)
            self.current_state = "EXECUTING"

            # Send goal to controller
            cmd_msg = String()
            cmd_msg.data = f"EXECUTE:{next_goal}"
            self.controller_cmd_pub.publish(cmd_msg)

    def is_goal_completed(self):
        # Check if the current goal has been completed
        # This is a simplified check - in practice, you'd check actual robot state
        return self.controller_status.get('completed', False)
```

## Performance Considerations

### Communication Frequency

Different types of communication require different frequencies:

- **High-frequency (100Hz+)**: Joint states, sensor data, velocity commands
- **Medium-frequency (10-50Hz)**: Position commands, odometry, basic feedback
- **Low-frequency (1-10Hz)**: High-level goals, status updates, configuration

### Quality of Service Settings

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class QoSAwareNode(Node):

    def __init__(self):
        super().__init__('qos_aware_node')

        # High-priority, reliable communication for safety-critical data
        reliable_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Best-effort for sensor data where some loss is acceptable
        sensor_qos = QoSProfile(
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.critical_pub = self.create_publisher(String, '/critical_data', reliable_qos)
        self.sensor_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
```

## Summary

Effective communication between AI logic and robot controllers is fundamental to creating capable robotic systems. By understanding the different communication patterns, safety considerations, and integration strategies, you can design systems that leverage the decision-making power of AI while maintaining the precision and reliability of low-level control. The examples provided demonstrate various approaches to connecting these different system components, from simple command-state loops to complex coordinated behaviors.
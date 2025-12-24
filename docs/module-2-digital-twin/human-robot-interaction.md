---
sidebar_position: 7
---

# Human-Robot Interaction in Simulation

## Introduction

Human-Robot Interaction (HRI) in simulation provides a safe and controlled environment to develop, test, and validate interaction paradigms before deploying them on real robots. Simulation allows for the exploration of various interaction modalities, user interface designs, and safety protocols without the risks associated with physical human-robot contact. This guide covers how to implement and test human-robot interaction scenarios in digital twin environments.

## Types of Human-Robot Interaction in Simulation

### 1. Teleoperation

Teleoperation allows humans to remotely control robots through various interfaces. Simulation provides a safe environment to test teleoperation interfaces:

```xml
<!-- Example world with teleoperation elements -->
<sdf version="1.6">
  <world name="teleop_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Robot that can be teleoperated -->
    <model name="teleop_robot">
      <pose>0 0 0.1 0 0 0</pose>
      <link name="base_link">
        <inertial>
          <mass>10.0</mass>
          <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
          <material><diffuse>0.2 0.6 1.0 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
        </collision>
      </link>

      <!-- Differential drive plugin for mobile base -->
      <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
        <command_topic>cmd_vel</command_topic>
        <odometry_topic>odom</odometry_topic>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>base_link</robot_base_frame>
        <publish_odom>true</publish_odom>
        <publish_odom_tf>true</publish_odom_tf>
        <publish_wheel_tf>false</publish_wheel_tf>
        <wheel_separation>0.4</wheel_separation>
        <wheel_diameter>0.15</wheel_diameter>
        <max_wheel_torque>20.0</max_wheel_torque>
        <max_wheel_acceleration>1.0</max_wheel_acceleration>
      </plugin>
    </model>

    <!-- Objects for interaction -->
    <model name="interactive_object">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="object_link">
        <inertial>
          <mass>0.5</mass>
          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia>
        </inertial>
        <visual name="visual">
          <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
          <material><diffuse>1.0 0.5 0.0 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### 2. Voice Command Simulation

Simulating voice command systems allows for testing natural language interfaces:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import speech_recognition as sr
import pyttsx3
import threading
import time

class VoiceCommandSimulator(Node):
    """
    Simulates voice command processing for human-robot interaction.
    In real applications, this would interface with actual speech recognition.
    """

    def __init__(self):
        super().__init__('voice_command_simulator')

        # Publisher for robot commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for voice feedback
        self.speech_pub = self.create_publisher(String, '/robot_speech', 10)

        # Subscriber for laser scan to detect obstacles
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Voice command processing
        self.last_scan = None
        self.speech_engine = pyttsx3.init()

        # Start voice command processing in a separate thread
        self.voice_thread = threading.Thread(target=self.process_voice_commands)
        self.voice_thread.daemon = True
        self.voice_thread.start()

    def scan_callback(self, msg):
        """Update last scan data"""
        self.last_scan = msg

    def process_voice_commands(self):
        """Simulate voice command processing"""
        # In simulation, we'll use predefined commands instead of real speech recognition
        # This simulates what would happen with actual voice input

        commands = [
            "move forward",
            "turn left",
            "stop",
            "go to the kitchen"
        ]

        for command in commands:
            self.get_logger().info(f"Processing command: {command}")
            self.execute_command(command)
            time.sleep(3)  # Wait between commands

    def execute_command(self, command):
        """Execute voice command"""
        cmd = Twist()

        if "forward" in command:
            cmd.linear.x = 0.5
            self.get_logger().info("Moving forward")
        elif "backward" in command:
            cmd.linear.x = -0.5
            self.get_logger().info("Moving backward")
        elif "left" in command:
            cmd.angular.z = 0.5
            self.get_logger().info("Turning left")
        elif "right" in command:
            cmd.angular.z = -0.5
            self.get_logger().info("Turning right")
        elif "stop" in command:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Stopping")
        else:
            self.get_logger().info(f"Unknown command: {command}")
            return

        # Check for obstacles before moving
        if self.last_scan and self.check_obstacles():
            self.get_logger().warn("Obstacle detected, stopping")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            # Publish warning
            warning_msg = String()
            warning_msg.data = "Obstacle detected, cannot move forward"
            self.speech_pub.publish(warning_msg)
        else:
            self.cmd_pub.publish(cmd)

    def check_obstacles(self):
        """Check if there are obstacles in front of the robot"""
        if not self.last_scan:
            return False

        # Check the front 30 degrees for obstacles
        front_range = len(self.last_scan.ranges) // 2
        scan_width = len(self.last_scan.ranges) // 12  # ~30 degrees

        for i in range(front_range - scan_width, front_range + scan_width):
            if i >= 0 and i < len(self.last_scan.ranges):
                if self.last_scan.ranges[i] < 0.5:  # Obstacle within 50cm
                    return True
        return False

def main(args=None):
    rclpy.init(args=args)
    voice_simulator = VoiceCommandSimulator()

    try:
        rclpy.spin(voice_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        voice_simulator.destroy_node()
        rclpy.shutdown()
```

### 3. Gesture Recognition Simulation

Simulating gesture recognition for intuitive human-robot interaction:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class GestureRecognitionSimulator(Node):
    """
    Simulates gesture recognition for human-robot interaction.
    In simulation, this processes camera images to detect gestures.
    """

    def __init__(self):
        super().__init__('gesture_recognition_simulator')

        # Publisher for robot commands
        self.cmd_pub = self.create_publisher(Twift, '/cmd_vel', 10)

        # Publisher for gesture recognition feedback
        self.gesture_pub = self.create_publisher(String, '/gesture_recognition', 10)

        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
        self.gesture_history = []

    def image_callback(self, msg):
        """Process camera image for gesture recognition"""
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process the image to detect gestures
            gesture = self.detect_gesture(cv_image)

            if gesture:
                self.get_logger().info(f"Detected gesture: {gesture}")

                # Execute corresponding action
                self.execute_gesture_action(gesture)

                # Publish gesture recognition result
                gesture_msg = String()
                gesture_msg.data = gesture
                self.gesture_pub.publish(gesture_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def detect_gesture(self, image):
        """Detect gesture from image (simplified for simulation)"""
        # In a real implementation, this would use computer vision
        # or machine learning to detect gestures

        # For simulation, we'll use a simplified approach
        # that detects hand positions in a specific area

        # Define ROI for gesture detection (simplified)
        height, width = image.shape[:2]
        roi = image[height//2:, width//4:3*width//4]  # Bottom half, middle third

        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define range for skin color (simplified)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)

        # Create mask
        mask = cv2.inRange(hsv, lower_skin, upper_skin)

        # Apply morphological operations to clean up the mask
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        mask = cv2.GaussianBlur(mask, (3, 3), 0)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > 5000:  # Minimum area threshold
                # Calculate the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Determine gesture based on position
                    roi_width = roi.shape[1]
                    if cx < roi_width * 0.33:
                        return "left"
                    elif cx > roi_width * 0.66:
                        return "right"
                    else:
                        return "forward"

        return None

    def execute_gesture_action(self, gesture):
        """Execute action based on detected gesture"""
        cmd = Twist()

        if gesture == "forward":
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        elif gesture == "left":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3
        elif gesture == "right":
            cmd.linear.x = 0.0
            cmd.angular.z = -0.3
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    gesture_simulator = GestureRecognitionSimulator()

    try:
        rclpy.spin(gesture_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_simulator.destroy_node()
        rclpy.shutdown()
```

## Safety in Human-Robot Interaction Simulation

### Safety Zones and Boundaries

```xml
<!-- Example with safety zones -->
<world name="safe_hri_world">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <gravity>0 0 -9.8</gravity>
  </physics>

  <!-- Human model (for simulation purposes) -->
  <model name="human">
    <pose>3 0 0.75 0 0 0</pose>
    <link name="human_link">
      <visual name="visual">
        <geometry><cylinder><radius>0.2</radius><length>1.5</length></cylinder></geometry>
        <material><diffuse>0.8 0.2 0.8 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.2</radius><length>1.5</length></cylinder></geometry>
      </collision>
      <inertial>
        <mass>70.0</mass>
        <inertia><ixx>10.0</ixx><iyy>10.0</iyy><izz>5.0</izz></inertia>
      </inertial>
    </link>
  </model>

  <!-- Robot with safety awareness -->
  <model name="safe_robot">
    <pose>0 0 0.1 0 0 0</pose>
    <link name="base_link">
      <visual name="visual">
        <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
        <material><diffuse>0.2 0.6 1.0 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
      </collision>
    </link>

    <!-- Safety zone (invisible) -->
    <model name="safety_zone">
      <static>true</static>
      <link name="zone_link">
        <visual name="visual">
          <geometry><sphere><radius>1.0</radius></sphere></geometry>
          <material><diffuse>1.0 0.0 0.0 0.2</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><sphere><radius>1.0</radius></sphere></geometry>
        </collision>
      </link>
      <pose>0 0 0.5 0 0 0</pose>
    </model>
  </model>
</world>
```

### Safety Monitoring Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
import math

class SafetyMonitor(Node):
    """
    Monitors safety in human-robot interaction scenarios.
    """

    def __init__(self):
        super().__init__('safety_monitor')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.human_pose_sub = self.create_subscription(
            PoseStamped, '/human_pose', self.human_pose_callback, 10)

        # Publishers
        self.safety_pub = self.create_publisher(Bool, '/safety_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.warning_pub = self.create_publisher(String, '/safety_warning', 10)

        # Parameters
        self.safety_distance = 1.0  # meters
        self.human_position = None
        self.robot_position = None

        # Timer for safety checks
        self.timer = self.create_timer(0.1, self.safety_check)

    def scan_callback(self, msg):
        """Process laser scan to detect humans or obstacles"""
        # In simulation, we might get human detection from other nodes
        # This is a simplified version that checks for close obstacles
        min_distance = min([r for r in msg.ranges if not math.isnan(r) and r > 0], default=float('inf'))

        if min_distance < self.safety_distance:
            self.trigger_safety_alert(f"Obstacle detected at {min_distance:.2f}m, within safety distance of {self.safety_distance}m")

    def human_pose_callback(self, msg):
        """Update human position"""
        self.human_position = msg.pose.position

    def safety_check(self):
        """Check safety conditions"""
        safety_status = Bool()
        safety_status.data = True  # Assume safe by default

        if self.human_position:
            # Calculate distance to human
            distance = math.sqrt(
                self.human_position.x**2 +
                self.human_position.y**2 +
                self.human_position.z**2
            )

            if distance < self.safety_distance:
                safety_status.data = False
                self.trigger_safety_alert(f"Human detected at {distance:.2f}m, within safety distance of {self.safety_distance}m")

        self.safety_pub.publish(safety_status)

    def trigger_safety_alert(self, message):
        """Trigger safety alert and emergency stop if needed"""
        self.get_logger().warn(message)

        # Publish warning
        warning_msg = String()
        warning_msg.data = message
        self.warning_pub.publish(warning_msg)

        # Publish emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()

    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()
```

## User Interface Simulation

### Graphical User Interface for HRI

```python
import tkinter as tk
from tkinter import ttk
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading

class HRIGUI:
    """
    Graphical user interface for human-robot interaction simulation.
    """

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Human-Robot Interaction Simulator")
        self.root.geometry("400x300")

        # Publisher for robot commands
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

        # Create GUI elements
        self.create_widgets()

        # Start GUI in separate thread
        self.gui_thread = threading.Thread(target=self.run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

    def create_widgets(self):
        """Create GUI widgets"""
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Title
        title_label = ttk.Label(main_frame, text="Robot Control Interface", font=("Arial", 14, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=10)

        # Direction buttons
        self.up_btn = ttk.Button(main_frame, text="↑ Forward", command=self.move_forward)
        self.up_btn.grid(row=1, column=1, pady=5)

        self.left_btn = ttk.Button(main_frame, text="← Left", command=self.move_left)
        self.left_btn.grid(row=2, column=0, padx=5, pady=5)

        self.stop_btn = ttk.Button(main_frame, text="● Stop", command=self.stop_robot)
        self.stop_btn.grid(row=2, column=1, pady=5)

        self.right_btn = ttk.Button(main_frame, text="→ Right", command=self.move_right)
        self.right_btn.grid(row=2, column=2, padx=5, pady=5)

        self.down_btn = ttk.Button(main_frame, text="↓ Backward", command=self.move_backward)
        self.down_btn.grid(row=3, column=1, pady=5)

        # Speed control
        speed_frame = ttk.Frame(main_frame)
        speed_frame.grid(row=4, column=0, columnspan=3, pady=10)

        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT)
        self.speed_var = tk.DoubleVar(value=0.5)
        speed_scale = ttk.Scale(speed_frame, from_=0.1, to=1.0, variable=self.speed_var, orient=tk.HORIZONTAL)
        speed_scale.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        # Voice command simulation
        voice_frame = ttk.Frame(main_frame)
        voice_frame.grid(row=5, column=0, columnspan=3, pady=10)

        ttk.Label(voice_frame, text="Voice Command:").pack(anchor=tk.W)

        self.voice_entry = ttk.Entry(voice_frame, width=30)
        self.voice_entry.pack(side=tk.LEFT, padx=5)
        self.voice_entry.bind('<Return>', self.process_voice_command)

        voice_btn = ttk.Button(voice_frame, text="Send", command=self.process_voice_command)
        voice_btn.pack(side=tk.LEFT)

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = self.speed_var.get()
        self.cmd_pub.publish(cmd)

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -self.speed_var.get()
        self.cmd_pub.publish(cmd)

    def move_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.angular.z = self.speed_var.get()
        self.cmd_pub.publish(cmd)

    def move_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.angular.z = -self.speed_var.get()
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Stop robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def process_voice_command(self, event=None):
        """Process voice command from entry"""
        command = self.voice_entry.get()
        self.node.get_logger().info(f"Processing voice command: {command}")

        # Clear entry
        self.voice_entry.delete(0, tk.END)

        # Execute command
        if "forward" in command.lower():
            self.move_forward()
        elif "backward" in command.lower():
            self.move_backward()
        elif "left" in command.lower():
            self.move_left()
        elif "right" in command.lower():
            self.move_right()
        elif "stop" in command.lower():
            self.stop_robot()

    def run_gui(self):
        """Run GUI main loop"""
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('hri_gui_node')

    # Create GUI
    gui = HRIGUI(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Integration with ROS 2 and Gazebo

### Connecting Simulation to Real Interaction

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener
import math

class HRIIntegrationNode(Node):
    """
    Integrates various HRI components in simulation.
    """

    def __init__(self):
        super().__init__('hri_integration_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/hri_markers', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.voice_sub = self.create_subscription(String, '/voice_commands', self.voice_callback, 10)
        self.safety_sub = self.create_subscription(Bool, '/safety_status', self.safety_callback, 10)

        # TF listener for transforms
        self.tf_buffer = rclpy.buffer.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables
        self.safety_status = True
        self.last_voice_command = ""

        # Timer for periodic updates
        self.timer = self.create_timer(0.1, self.update)

    def scan_callback(self, msg):
        """Process laser scan data"""
        # Process scan data for navigation and safety
        pass

    def image_callback(self, msg):
        """Process image data for gesture recognition"""
        # Process image data for gesture recognition
        pass

    def imu_callback(self, msg):
        """Process IMU data for robot state"""
        # Process IMU data for orientation and stability
        pass

    def voice_callback(self, msg):
        """Process voice commands"""
        self.last_voice_command = msg.data
        self.get_logger().info(f"Received voice command: {msg.data}")

    def safety_callback(self, msg):
        """Update safety status"""
        self.safety_status = msg.data
        if not self.safety_status:
            self.get_logger().warn("Safety system indicates unsafe conditions")

    def update(self):
        """Periodic update function"""
        # Update visualization markers
        self.publish_markers()

    def publish_markers(self):
        """Publish visualization markers for HRI"""
        # Publish markers for safety zones, interaction areas, etc.
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "hri"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set position (example: safety zone)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set scale
        marker.scale.x = 2.0  # Safety radius
        marker.scale.y = 2.0
        marker.scale.z = 0.1  # Flat marker

        # Set color (red for safety zone)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.3  # Semi-transparent

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    hri_node = HRIIntegrationNode()

    try:
        rclpy.spin(hri_node)
    except KeyboardInterrupt:
        pass
    finally:
        hri_node.destroy_node()
        rclpy.shutdown()
```

### Connecting Simulation to ROS 2 Systems

To connect Gazebo simulation to ROS 2 systems, several key components must be properly configured:

#### 1. Gazebo ROS Plugins

Gazebo uses plugins to interface with ROS 2. Common plugins include:

- **libgazebo_ros_diff_drive.so**: For differential drive robots
- **libgazebo_ros_camera.so**: For camera sensors
- **libgazebo_ros_laser.so**: For LiDAR sensors
- **libgazebo_ros_imu.so**: For IMU sensors
- **libgazebo_ros_p3d.so**: For pose/position tracking

Example configuration in SDF:
```xml
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <publish_wheel_tf>false</publish_wheel_tf>
  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.15</wheel_diameter>
</plugin>
```

#### 2. Topic Mapping

Ensure proper topic mapping between simulation and ROS 2 nodes:

```python
# In your ROS 2 node
class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Publishers - match topics defined in Gazebo plugins
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribers - match topics published by Gazebo
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
```

#### 3. Coordinate Frame Management

Use TF (Transform) to manage coordinate frames between simulation and real world:

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish transforms to connect simulation frames
        self.timer = self.create_timer(0.05, self.broadcast_transforms)

    def broadcast_transforms(self):
        """Broadcast coordinate transforms"""
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Get robot pose from simulation or odometry
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
```

#### 4. Launch Configuration

Create launch files to start both Gazebo and ROS 2 nodes together:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # World file argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_gazebo'),
            'worlds',
            'hri_world.sdf'
        ]),
        description='SDF world file'
    )

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )

    # Launch robot controller node
    controller_node = Node(
        package='my_robot_control',
        executable='controller_node',
        name='robot_controller',
        output='screen'
    )

    # Launch HRI interface node
    hri_node = Node(
        package='my_robot_hri',
        executable='hri_interface',
        name='hri_interface',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        controller_node,
        hri_node
    ])
```

#### 5. Simulation vs. Real Robot Configuration

Use parameter files to switch between simulation and real robot configurations:

```yaml
# simulation_params.yaml
robot_controller:
  ros__parameters:
    use_sim_time: true
    cmd_vel_topic: "/cmd_vel"
    odom_topic: "/odom"
    laser_topic: "/scan"
    camera_topic: "/camera/image_raw"

# real_robot_params.yaml
robot_controller:
  ros__parameters:
    use_sim_time: false
    cmd_vel_topic: "/hardware/cmd_vel"
    odom_topic: "/hardware/odom"
    laser_topic: "/hardware/scan"
    camera_topic: "/hardware/camera/image_raw"
```

This approach allows the same ROS 2 nodes to work in both simulation and real environments with minimal changes.

## Testing HRI Scenarios

### Scenario Testing Framework

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class HRITestFramework(Node):
    """
    Framework for testing HRI scenarios in simulation.
    """

    def __init__(self):
        super().__init__('hri_test_framework')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_pub = self.create_publisher(String, '/voice_commands', 10)
        self.test_result_pub = self.create_publisher(String, '/test_results', 10)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.safety_sub = self.create_subscription(Bool, '/safety_status', self.safety_callback, 10)

        # Test state
        self.current_test = 0
        self.tests = [
            self.test_voice_navigation,
            self.test_obstacle_avoidance,
            self.test_safety_response,
            self.test_gesture_recognition
        ]

        # Start testing
        self.timer = self.create_timer(1.0, self.run_tests)
        self.test_start_time = time.time()

    def scan_callback(self, msg):
        """Process scan data for tests"""
        self.last_scan = msg

    def safety_callback(self, msg):
        """Process safety status for tests"""
        self.safety_status = msg.data

    def run_tests(self):
        """Run HRI tests sequentially"""
        if self.current_test < len(self.tests):
            test_func = self.tests[self.current_test]
            self.get_logger().info(f"Running test {self.current_test + 1}: {test_func.__name__}")

            # Run the test
            result = test_func()

            # Publish result
            result_msg = String()
            result_msg.data = f"Test {self.current_test + 1} ({test_func.__name__}): {result}"
            self.test_result_pub.publish(result_msg)

            self.current_test += 1
        else:
            # All tests completed
            self.get_logger().info("All HRI tests completed")
            self.timer.cancel()

    def test_voice_navigation(self):
        """Test voice command navigation"""
        try:
            # Send voice command
            cmd = String()
            cmd.data = "move forward 2 meters"
            self.voice_pub.publish(cmd)

            # Wait for robot to move
            time.sleep(3)

            # Check if robot moved appropriately
            # (In simulation, we'd check position, in this example we just return success)
            return "PASSED"
        except Exception as e:
            return f"FAILED: {str(e)}"

    def test_obstacle_avoidance(self):
        """Test obstacle avoidance during interaction"""
        try:
            # Send movement command
            cmd = Twist()
            cmd.linear.x = 0.5
            self.cmd_pub.publish(cmd)

            # Wait and check if safety system activates
            time.sleep(2)

            # Stop robot
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            self.cmd_pub.publish(stop_cmd)

            return "PASSED"
        except Exception as e:
            return f"FAILED: {str(e)}"

    def test_safety_response(self):
        """Test safety system response"""
        try:
            # This would test the safety system in a real scenario
            # For simulation, we check if safety status is being monitored
            if hasattr(self, 'safety_status'):
                return "PASSED"
            else:
                return "FAILED: Safety status not available"
        except Exception as e:
            return f"FAILED: {str(e)}"

    def test_gesture_recognition(self):
        """Test gesture recognition (simulated)"""
        try:
            # In simulation, this would trigger gesture recognition
            # For now, we just return PASSED
            return "PASSED"
        except Exception as e:
            return f"FAILED: {str(e)}"

def main(args=None):
    rclpy.init(args=args)
    test_framework = HRITestFramework()

    try:
        rclpy.spin(test_framework)
    except KeyboardInterrupt:
        pass
    finally:
        test_framework.destroy_node()
        rclpy.shutdown()
```

## Best Practices for HRI Simulation

### 1. Realistic Human Modeling

- Model human behavior patterns realistically
- Include reaction times and decision-making delays
- Consider human comfort zones and safety preferences

### 2. Gradual Complexity Increase

- Start with simple interaction scenarios
- Gradually add complexity (multiple humans, dynamic environments)
- Test each level before proceeding

### 3. Comprehensive Safety Testing

- Test all safety scenarios thoroughly
- Include edge cases and unexpected situations
- Validate safety system responses

### 4. Multi-Modal Interaction

- Combine different interaction modalities (voice, gesture, touch)
- Test cross-modal consistency
- Ensure fallback mechanisms

## Summary

Human-Robot Interaction simulation provides a crucial testing ground for developing safe and effective interaction paradigms. By simulating various interaction modalities, safety systems, and user interfaces, we can validate HRI concepts before deploying them on physical robots. The examples provided demonstrate how to implement teleoperation, voice commands, gesture recognition, safety monitoring, and comprehensive testing frameworks in simulation environments. These approaches enable the development of intuitive, safe, and effective human-robot interaction systems for Physical AI applications.
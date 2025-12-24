# Sensor Simulation Example

This example demonstrates how to simulate different types of sensors on a robot in Gazebo: camera, LiDAR, and IMU sensors.

## Overview

The example includes a robot equipped with three different sensors and a test environment with various objects to sense:

- **Camera**: Visual sensor providing RGB images
- **LiDAR**: Range sensor providing 360-degree distance measurements
- **IMU**: Inertial measurement unit providing acceleration and angular velocity

## Running the Example

1. **Launch Gazebo with the sensor world:**
```bash
gazebo --verbose sensor_world.sdf
```

2. **In another terminal, subscribe to sensor topics to see the data:**

For camera data:
```bash
# View camera images
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

For LiDAR data:
```bash
# Echo laser scan data
ros2 topic echo /scan sensor_msgs/msg/LaserScan
```

For IMU data:
```bash
# Echo IMU data
ros2 topic echo /imu sensor_msgs/msg/Imu
```

For robot movement:
```bash
# Send velocity commands to move the robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

## Sensor Configuration

### Camera Sensor
- **Field of View**: 60 degrees horizontal
- **Resolution**: 640×480 pixels
- **Format**: RGB color
- **Range**: 0.1m to 10m
- **Topic**: `/camera/image_raw`
- **Frame**: `camera_frame`

### LiDAR Sensor
- **Samples**: 360 (1 degree resolution)
- **Range**: 0.1m to 10m
- **Field of View**: 360 degrees horizontal
- **Topic**: `/scan`
- **Frame**: `laser_frame`

### IMU Sensor
- **Measurements**: Linear acceleration and angular velocity
- **Noise**: Realistic Gaussian noise models
- **Topic**: `/imu`
- **Frame**: `imu_frame`

## Robot Configuration

The robot uses differential drive for movement:
- **Command Topic**: `/cmd_vel`
- **Odometry Topic**: `/odom`
- **Wheel Separation**: 0.4m
- **Wheel Diameter**: 0.15m

## Test Environment

The environment includes various objects for sensor testing:
- **Box Obstacle**: Red box for camera and LiDAR detection
- **Cylinder Obstacle**: Green cylinder for shape recognition
- **Sphere Obstacle**: Blue sphere for 3D detection
- **Wall**: Long wall for LiDAR mapping
- **Small Boxes**: Small objects for detailed sensing

## Sensor Data Processing Example

Here's a simple Python example to process the sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from cv_bridge import CvBridge
import cv2

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')
        self.bridge = CvBridge()

        # Subscribers for all sensors
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

    def camera_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Process image (example: detect edges)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Display
        cv2.imshow('Camera Image', cv_image)
        cv2.imshow('Edges', edges)
        cv2.waitKey(1)

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = msg.ranges
        min_range = min(r for r in ranges if r > 0 and r < float('inf'))

        if min_range < 0.5:  # Obstacle within 50cm
            self.get_logger().warn(f'Obstacle detected at {min_range:.2f}m')

    def imu_callback(self, msg):
        # Process IMU data
        linear_acc = msg.linear_acceleration
        angular_vel = msg.angular_velocity

        self.get_logger().info(
            f'Linear: ({linear_acc.x:.2f}, {linear_acc.y:.2f}, {linear_acc.z:.2f}), '
            f'Angular: ({angular_vel.x:.2f}, {angular_vel.y:.2f}, {angular_vel.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    processor = SensorProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()
```

## Customization

You can modify the sensor parameters in the SDF file:

### Camera Parameters
```xml
<camera name="front_camera">
  <horizontal_fov>1.047</horizontal_fov> <!-- Change field of view -->
  <image>
    <width>1280</width> <!-- Change resolution -->
    <height>720</height>
  </image>
  <noise>
    <stddev>0.01</stddev> <!-- Change noise level -->
  </noise>
</camera>
```

### LiDAR Parameters
```xml
<ray>
  <scan>
    <horizontal>
      <samples>720</samples> <!-- Increase resolution -->
      <min_angle>-3.14</min_angle> <!-- 180 degrees -->
      <max_angle>3.14</max_angle>  <!-- 180 degrees -->
    </horizontal>
  </scan>
  <range>
    <max>30.0</max> <!-- Increase max range -->
  </range>
</ray>
```

## Integration with ROS 2

The sensors are configured to work with ROS 2 using the Gazebo ROS packages:
- Camera publishes to `/camera/image_raw` and `/camera/camera_info`
- LiDAR publishes to `/scan`
- IMU publishes to `/imu`
- Differential drive accepts commands on `/cmd_vel` and publishes to `/odom`

## Troubleshooting

1. **No sensor data**: Check that Gazebo ROS plugins are installed
2. **Slow performance**: Reduce sensor resolution or update rate
3. **Noisy data**: Adjust noise parameters in SDF file
4. **Missing topics**: Verify that the correct Gazebo plugins are loaded

## Learning Outcomes

This example demonstrates:
- How to configure different types of sensors in Gazebo
- How sensor data is published to ROS 2 topics
- How to process sensor data in ROS 2 nodes
- How to create a test environment for sensor validation
- The importance of realistic noise modeling in simulation

This foundation is essential for developing perception systems in Digital Twin environments for Physical AI applications.
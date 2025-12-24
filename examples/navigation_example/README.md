# Simple Isaac ROS Navigation Example

This example demonstrates basic navigation using Nav2 and Isaac ROS integration. The example includes waypoint following, obstacle avoidance, basic path planning, and integration with Isaac ROS perception systems.

## Overview

The navigation example implements:
1. Waypoint following using Nav2 navigation stack
2. Obstacle detection and avoidance
3. Basic path planning and execution
4. Integration with Isaac ROS perception systems
5. Safety considerations and emergency handling

## Components

### Main Navigation Node (`simple_navigation.py`)

The main navigation node implements:
- Waypoint following with Nav2 integration
- Obstacle detection using laser scan data
- Navigation safety and emergency stopping
- Standard ROS message interfaces

### Key Features

#### Waypoint Navigation
- Configurable waypoint lists
- Automatic progression through waypoints
- Distance-based arrival detection
- Navigation status reporting

#### Obstacle Avoidance
- Real-time obstacle detection from laser scan
- Configurable obstacle distance threshold
- Emergency stopping when obstacles detected
- Integration with Nav2 recovery behaviors

#### Safety Systems
- Collision prevention
- Navigation status monitoring
- Emergency stop capabilities
- Graceful failure handling

## Prerequisites

Before running the example, ensure you have:

1. **ROS 2 with Navigation2**:
```bash
# Install Navigation2
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

2. **Isaac ROS Navigation Components** (optional but recommended):
```bash
# Install Isaac ROS navigation packages
# This typically involves building from source
```

3. **Robot Drivers**:
- Odometry publisher
- Laser scanner (LiDAR)
- Velocity command subscriber
- TF broadcasters for robot transforms

## Installation

1. **Create a ROS 2 package for the example:**
```bash
mkdir -p ~/navigation_example_ws/src
cd ~/navigation_example_ws/src
ros2 pkg create --build-type ament_python navigation_example
```

2. **Copy the example files:**
```bash
# Copy the navigation script
cp simple_navigation.py ~/navigation_example_ws/src/navigation_example/navigation_example/

# Copy the launch file
cp launch_navigation.py ~/navigation_example_ws/src/navigation_example/launch/
```

3. **Build the workspace:**
```bash
cd ~/navigation_example_ws
colcon build --packages-select navigation_example
source install/setup.bash
```

## Running the Example

### With Real Robot

1. **Start your robot drivers:**
```bash
# Launch your robot bringup
ros2 launch your_robot_bringup robot.launch.py
```

2. **Run the navigation example:**
```bash
ros2 run navigation_example simple_navigation
```

### With Simulation

1. **Launch simulation environment:**
```bash
# Launch Gazebo with a robot that has navigation capabilities
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

2. **Launch navigation stack:**
```bash
# Launch Nav2 stack
ros2 launch nav2_bringup navigation_launch.py
```

3. **Run the navigation example:**
```bash
ros2 run navigation_example simple_navigation
```

### With Map Server

1. **Launch with a map:**
```bash
ros2 launch navigation_example launch_navigation.py map_file:=/path/to/your/map.yaml
```

## Topics and Messages

### Published Topics

- `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands for robot
- `/navigation_status` (`std_msgs/Bool`): Navigation active status
- TF transforms: Robot pose transforms

### Subscribed Topics

- `/odom` (`nav_msgs/Odometry`): Robot odometry data
- `/scan` (`sensor_msgs/LaserScan`): Laser scan data for obstacle detection
- `/navigate_to_pose` (action): Navigation goals from Nav2

## Parameters

The navigation system has several configurable parameters:

- `linear_speed`: Robot linear speed (default: 0.3 m/s)
- `angular_speed`: Robot angular speed (default: 0.5 rad/s)
- `arrival_threshold`: Distance threshold for reaching waypoint (default: 0.5m)
- `min_obstacle_distance`: Minimum distance for obstacle detection (default: 0.5m)
- `use_sim_time`: Whether to use simulation time (default: false)

## Integration with Isaac ROS

This example demonstrates integration with Isaac ROS concepts:

### 1. Perception Integration
The navigation system can be enhanced with Isaac ROS perception:
- Object detection for dynamic obstacle avoidance
- Semantic segmentation for traversable area detection
- Depth estimation for 3D navigation

### 2. Planning Integration
Nav2 can be extended with Isaac ROS planning capabilities:
- Visual SLAM for map building
- Semantic mapping for contextual navigation
- Deep learning for path planning

### 3. Control Integration
Isaac ROS control components can enhance navigation:
- GPU-accelerated trajectory optimization
- Learning-based control policies
- Model predictive control

## Isaac ROS Perception Integration

To integrate with Isaac ROS perception, you can extend the navigation system:

```python
# Example of Isaac ROS perception integration
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

class EnhancedNavigation(SimpleNavigation):
    def __init__(self):
        super().__init__()

        # Isaac ROS perception integration
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/isaac_ros/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        """Process Isaac ROS detections"""
        for detection in msg.detections:
            # Handle detected objects
            if detection.results[0].hypothesis.class_id == 'person':
                # Adjust navigation for people
                self.avoid_person(detection.bbox)

    def avoid_person(self, bbox):
        """Avoid detected person"""
        # Implement person avoidance logic
        pass
```

## Safety Considerations

### Emergency Procedures
- Emergency stop when obstacles are too close
- Graceful degradation when sensors fail
- Manual override capabilities
- Collision prevention algorithms

### Operational Limits
- Maximum speed limits
- Operational area boundaries
- Sensor range limitations
- Processing time constraints

## Extending the Example

### Adding Advanced Features

1. **Semantic Navigation:**
```python
# Integrate semantic information for contextual navigation
def navigate_with_context(self, semantic_map):
    # Use semantic information to plan better paths
    pass
```

2. **Learning-based Navigation:**
```python
# Implement learning-based navigation policies
def learn_navigation_policy(self, experience_data):
    # Train neural network for navigation
    pass
```

### Performance Optimization

1. **Multi-threading:**
```python
# Run perception and navigation in separate threads
import threading

def run_perception_thread(self):
    # Run perception pipeline in background
    pass
```

2. **GPU Acceleration:**
```python
# Use GPU for intensive computations
import cupy as cp

def gpu_path_planning(self, map_data):
    # Accelerate path planning with GPU
    pass
```

## Performance Considerations

### Real-time Performance

The navigation system is designed for real-time performance:
- Efficient obstacle detection algorithms
- Optimized path planning
- Asynchronous message handling
- Predictive control for smooth motion

### Scalability

The system can be scaled for different requirements:
- Multiple robot coordination
- Large map handling
- High-frequency updates
- Complex environment navigation

## Troubleshooting

### Common Issues

1. **Navigation Stuck**
   - Check costmap inflation parameters
   - Verify localization accuracy
   - Check for sensor issues
   - Review path planning parameters

2. **Oscillation**
   - Increase controller lookahead distance
   - Adjust PID parameters
   - Smooth path with spline interpolation
   - Check velocity limits

3. **Obstacle Detection Issues**
   - Verify sensor calibration
   - Check sensor noise parameters
   - Adjust detection thresholds
   - Validate sensor placement

### Debugging

Enable debug output to troubleshoot navigation issues:
```bash
# Run with debug logging
ros2 run navigation_example simple_navigation --ros-args -p __log_level:=DEBUG

# Monitor navigation topics
ros2 topic echo /navigate_to_pose/_action/status
```

## Related Isaac ROS Packages

This example works with several Isaac ROS packages:
- `isaac_ros_visual_slam`: For map building and localization
- `isaac_ros_detectnet`: For object detection in navigation
- `isaac_ros_point_cloud_utils`: For 3D navigation planning
- `isaac_ros_manipulation`: For navigation with manipulation tasks

## Configuration Files

### Costmap Parameters

The example uses standard Nav2 costmap configurations:
- Local costmap for obstacle avoidance
- Global costmap for path planning
- Inflation layers for safety margins

### Behavior Trees

Navigation behavior trees control the overall navigation flow:
- Path planning and execution
- Recovery behaviors
- Goal management
- State monitoring

## License

This example is provided under the Apache 2.0 license. See the license file for details.
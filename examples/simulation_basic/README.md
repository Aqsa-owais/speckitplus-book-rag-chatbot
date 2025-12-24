# Basic Physics Simulation Example

This example demonstrates fundamental physics simulation concepts in Gazebo including gravity, collisions, constraints, and dynamic objects.

## Overview

The physics simulation world includes several objects that demonstrate different physical behaviors:

- **Static Box**: A fixed object that doesn't move
- **Falling Sphere**: A dynamic object that falls due to gravity
- **Ramp and Sliding Box**: Objects that demonstrate sliding motion
- **Pendulum**: A constrained system with a revolute joint

## Running the Example

1. **Launch Gazebo with the physics world:**
```bash
gzserver --verbose physics_world.sdf
```

2. **In another terminal, launch the GUI:**
```bash
gzclient
```

Or run both together:
```bash
gazebo --verbose physics_world.sdf
```

## Physics Concepts Demonstrated

### 1. Gravity
The world demonstrates the effect of gravity on objects:
- The falling sphere accelerates downward at 9.8 m/s²
- All objects are affected by the gravitational field

### 2. Collisions
The simulation shows how objects interact when they collide:
- The falling sphere will hit the ground and bounce
- The sliding box will interact with the ramp surface

### 3. Constraints
The pendulum demonstrates constrained motion:
- The revolute joint allows rotation around a single axis
- Joint limits prevent excessive movement

### 4. Inertial Properties
Each object has realistic inertial properties:
- Mass affects how objects respond to forces
- Inertia affects rotational motion

## World Configuration

### Physics Parameters
- **Gravity**: 0 0 -9.8 m/s² (Earth gravity)
- **Time Step**: 0.001 seconds
- **Real Time Factor**: 1.0 (simulation runs at real-time speed)

### Objects Description

#### Static Box
- **Position**: (2, 0, 0.5)
- **Size**: 1×1×1 meters
- **Mass**: 1.0 kg
- **Purpose**: Fixed reference object

#### Falling Sphere
- **Position**: (-2, 0, 3)
- **Radius**: 0.2 meters
- **Mass**: 0.5 kg
- **Purpose**: Demonstrates gravity and collision response

#### Ramp and Sliding Box
- **Ramp**: Tilted surface for objects to slide on
- **Sliding Box**: Small box that slides down the ramp
- **Purpose**: Demonstrates friction and sliding motion

#### Pendulum
- **Base**: Fixed anchor point
- **Ball**: Moving part connected by a revolute joint
- **Purpose**: Demonstrates constrained motion and oscillation

## Customization

You can modify the physics parameters in the SDF file:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller = more accurate but slower -->
  <real_time_factor>1.0</real_time_factor>  <!-- 1.0 = real-time, >1 = faster -->
  <gravity>0 0 -9.8</gravity>  <!-- Standard Earth gravity -->
</physics>
```

## Integration with ROS 2

To use this world with ROS 2, you can create a launch file:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('your_package'),
                    'worlds',
                    'physics_world.sdf'
                ])
            }.items()
        )
    ])
```

## Troubleshooting

1. **Simulation runs too slowly**: Try increasing the time step or reducing solver iterations
2. **Objects behave erratically**: Check mass and inertia values, adjust ERP/CFM parameters
3. **Objects pass through each other**: Reduce time step or enable continuous collision detection

## Learning Outcomes

This example helps you understand:
- How to set up a basic physics simulation in Gazebo
- The effect of gravity on different objects
- Collision detection and response
- Joint constraints and their effects
- The importance of proper inertial properties

This foundation is essential for creating more complex Digital Twin environments for Physical AI applications.
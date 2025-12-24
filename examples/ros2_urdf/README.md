# Humanoid Robot URDF Example

This example demonstrates a simple humanoid robot model using URDF (Unified Robot Description Format).

## Overview

The humanoid robot model includes:
- Torso (base_link)
- Head with neck joint
- Two arms with shoulder and elbow joints
- Two legs with hip and knee joints

## URDF Structure

The robot is defined with the following kinematic chain:

```
base_link (torso)
├── head (via neck_joint)
├── left_upper_arm (via left_shoulder_joint)
│   └── left_lower_arm (via left_elbow_joint)
├── right_upper_arm (via right_shoulder_joint)
│   └── right_lower_arm (via right_elbow_joint)
├── left_upper_leg (via left_hip_joint)
│   └── left_lower_leg (via left_knee_joint)
└── right_upper_leg (via right_hip_joint)
    └── right_lower_leg (via right_knee_joint)
```

## Key Components

### Links
- **base_link**: The main body/torso of the robot
- **head**: The head component
- **Arms**: Upper and lower arms for both left and right sides
- **Legs**: Upper and lower legs for both left and right sides

### Joints
- **neck_joint**: Revolute joint connecting head to torso
- **Shoulder joints**: Revolute joints for arm movement
- **Elbow joints**: Revolute joints for forearm movement
- **Hip joints**: Revolute joints connecting legs to torso
- **Knee joints**: Revolute joints for lower leg movement

## Physical Properties

Each link includes:
- **Mass**: Physical mass of the link
- **Inertia**: Inertial properties for physics simulation
- **Visual**: How the link appears in visualization
- **Collision**: Collision geometry for physics simulation

## Visualization

To visualize this robot model:

```bash
# Launch RViz with robot state publisher
ros2 run rviz2 rviz2

# Add a RobotModel display and set the robot description parameter
# Or use the command line:
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat simple_humanoid.urdf)'
```

## Usage in Simulation

This URDF can be used in simulation environments like Gazebo:

```bash
# Spawn the robot in Gazebo
ros2 run gazebo_ros spawn_entity.py -file simple_humanoid.urdf -entity simple_humanoid
```

## Customization

The URDF can be customized by:
- Adding more joints and links for fingers, toes, etc.
- Adjusting physical properties like mass and inertia
- Changing geometric shapes and sizes
- Adding materials and colors

This example provides a foundation for more complex humanoid robot models used in Physical AI applications.
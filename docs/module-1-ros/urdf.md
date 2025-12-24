---
sidebar_position: 6
---

# Humanoid Robot Description using URDF

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links, joints, inertial properties, visual meshes, and collision geometries. URDF is essential for simulation, visualization, and control of robotic systems in ROS environments.

## URDF Structure

A URDF file describes a robot as a collection of links connected by joints, forming a kinematic tree. The basic structure includes:

- **Links**: Rigid bodies with physical properties
- **Joints**: Connections between links with kinematic properties
- **Visual**: How the robot appears in visualization tools
- **Collision**: How the robot interacts in physics simulations
- **Inertial**: Mass, center of mass, and inertia properties

## Basic URDF Elements

### Links

A link represents a rigid body part of the robot. Each link can have:

- **Inertial properties**: Mass, center of mass, and inertia matrix
- **Visual properties**: How the link appears in visualizations
- **Collision properties**: How the link behaves in collision detection

```xml
<link name="link_name">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
</link>
```

### Joints

A joint connects two links with specific kinematic properties:

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

Joint types include:
- **revolute**: Rotational joint with limits
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint with limits
- **fixed**: No movement (welds links together)
- **floating**: 6-DOF joint
- **planar**: Movement in a plane

## Complete Humanoid Robot Example

Here's a simplified humanoid robot URDF with torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.5 0.3 1.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.5 0.3 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 1.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.8"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left) -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.2 0 0.8"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.1 0 0.1"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Right Leg (similar to left) -->
  <link name="right_upper_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 0.1"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>

</robot>
```

## URDF Best Practices

### 1. Proper Inertial Properties

Accurate inertial properties are crucial for physics simulation:

```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.083" ixy="0.0" ixz="0.0" iyy="0.083" iyz="0.0" izz="0.125"/>
</inertial>
```

For common shapes, the inertia values are:
- **Box** (width x, y, z): `ixx = m*(y²+z²)/12`, `iyy = m*(x²+z²)/12`, `izz = m*(x²+y²)/12`
- **Cylinder** (radius r, length l): `ixx = m*(3*r²+l²)/12`, `izz = m*r²/2`
- **Sphere** (radius r): `ixx = iyy = izz = 2*m*r²/5`

### 2. Using Xacro for Complex Models

For complex robots, use Xacro (XML Macros) to make URDF more maintainable:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_arm" params="side reflect:=1">
    <link name="${side}_upper_arm">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="0 0 -0.15"/>
        <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
      </inertial>
      <visual>
        <origin xyz="0 0 -0.15"/>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${reflect * 0.2} 0 0.8"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:simple_arm side="left" reflect="1" />
  <xacro:simple_arm side="right" reflect="-1" />

</robot>
```

### 3. Materials and Colors

Define materials separately for consistency:

```xml
<material name="black">
  <color rgba="0.0 0.0 0.0 1.0"/>
</material>

<material name="blue">
  <color rgba="0.0 0.0 0.8 1.0"/>
</material>

<material name="green">
  <color rgba="0.0 0.8 0.0 1.0"/>
</material>

<material name="grey">
  <color rgba="0.5 0.5 0.5 1.0"/>
</material>

<material name="orange">
  <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
</material>

<material name="brown">
  <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
</material>

<material name="red">
  <color rgba="0.8 0.0 0.0 1.0"/>
</material>

<material name="white">
  <color rgba="1.0 1.0 1.0 1.0"/>
</material>
```

## Validating URDF Models

### Using check_urdf

ROS provides a command-line tool to validate URDF files:

```bash
# Check the URDF for errors
check_urdf /path/to/robot.urdf

# Print the kinematic tree
urdf_to_graphiz /path/to/robot.urdf
```

### Using rviz for Visualization

```bash
# Launch rviz with robot state publisher
ros2 run rviz2 rviz2
```

In rviz, add a RobotModel display and set the robot description parameter to your URDF.

### Using Gazebo for Physics Validation

```bash
# Launch Gazebo with your robot
ros2 launch gazebo_ros gazebo.launch.py
ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf -entity robot_name
```

## Integration with ROS 2

### Robot State Publisher

The robot_state_publisher node reads the URDF and publishes the joint transforms:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        # Process joint states and publish transforms
        # This is handled automatically by robot_state_publisher
        pass
```

### Loading URDF in Launch Files

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments
    urdf_tutorial_path = os.path.join(
        get_package_share_path('my_robot_description'),
        'urdf',
        'my_robot.urdf'
    )

    params = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=urdf_tutorial_path,
                              description='Absolute path to robot urdf file'),
        node_robot_state_publisher
    ])
```

## Common URDF Issues and Solutions

### 1. Missing Joint Limits

Always specify joint limits for revolute and prismatic joints:

```xml
<!-- Good -->
<joint name="joint_with_limits" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>

<!-- Bad -->
<joint name="joint_without_limits" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <!-- No limits specified -->
</joint>
```

### 2. Incorrect Origins

Ensure joint origins are correctly specified relative to parent link:

```xml
<!-- Correct origin specification -->
<joint name="arm_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.8" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
</joint>
```

### 3. Massless Links

All links should have mass properties, even if small:

```xml
<!-- Good -->
<link name="sensor_link">
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Bad -->
<link name="sensor_link">
  <!-- No inertial properties -->
</link>
```

## Summary

URDF is a fundamental component of ROS-based robotics, providing a standardized way to describe robot models. By understanding the structure of URDF files, the relationships between links and joints, and best practices for creating robot descriptions, you can build accurate models for simulation, visualization, and control of humanoid robots. The examples provided demonstrate the key concepts needed to create your own robot descriptions, from simple single-link robots to complex multi-degree-of-freedom humanoid systems.
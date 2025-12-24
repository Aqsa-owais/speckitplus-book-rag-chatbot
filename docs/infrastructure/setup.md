---
sidebar_position: 8
---

# Development Environment Setup

This guide provides step-by-step instructions for setting up your development environment for Physical AI robotics development.

## Prerequisites

Before starting, ensure you have:
- Ubuntu 22.04 LTS installed
- Administrative (sudo) access to your machine
- Internet connection for downloading packages
- At least 50GB of free disk space
- Basic familiarity with command line operations

## Install Docusaurus Requirements

First, install Node.js and Yarn for building the documentation:

```bash
# Install Node.js (LTS version recommended)
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install Yarn
npm install -g yarn
```

## Install ROS 2 (Humble Hawksbill)

Set up your locale:

```bash
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
```

Add the ROS 2 apt repository:

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 packages:

```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Install Simulation Tools

Install Gazebo Garden:

```bash
sudo apt install ros-humble-gazebo-*
sudo apt install ignition-garden
```

For NVIDIA Isaac Sim, follow the specific installation guide from NVIDIA. This typically involves:
- Installing NVIDIA drivers (525 or later)
- Installing Docker and Docker Compose
- Downloading Isaac Sim from NVIDIA Developer portal
- Setting up the Isaac Sim Docker containers

## Set Up the Book Repository

Clone the repository:

```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Install Docusaurus dependencies
cd docs
yarn install

# For ROS 2 workspace (if needed for examples)
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws
source /opt/ros/humble/setup.bash
colcon build
```

## Environment Variables

Add ROS 2 to your shell profile:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Verify Installation

Test that ROS 2 is working:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Test basic functionality
ros2 topic list
```

## Common Issues and Solutions

### Docusaurus Build Issues
- Ensure all Markdown files have proper frontmatter
- Check for broken internal links
- Verify syntax highlighting language tags

### ROS 2 Environment Issues
- Always source the ROS 2 setup.bash file
- Check that packages are properly installed
- Verify Python path includes ROS 2 libraries

### Simulation Problems
- Ensure GPU drivers are properly installed for graphics-intensive simulators
- Check that simulation packages are correctly installed
- Verify network connectivity if using distributed simulation

## Next Steps

1. Complete the Introduction section to understand Physical AI foundations
2. Work through Module 1: The Robotic Nervous System (ROS 2)
3. Begin with basic ROS 2 examples and progress to more complex systems
4. Build toward the capstone autonomous humanoid project

Your development environment is now set up and ready for Physical AI robotics development!
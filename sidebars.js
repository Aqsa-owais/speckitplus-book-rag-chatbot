// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro/index',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros/index',
        'module-1-ros/concepts',
        'module-1-ros/python-ros',
        'module-1-ros/communication',
        'module-1-ros/urdf'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/physics-simulation',
        'module-2-digital-twin/sensor-simulation',
        'module-2-digital-twin/environments',
        'module-2-digital-twin/human-robot-interaction'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-ai-brain/index',
        'module-3-ai-brain/nvidia-isaac',
        'module-3-ai-brain/synthetic-data',
        'module-3-ai-brain/perception-pipelines',
        'module-3-ai-brain/slam-localization',
        'module-3-ai-brain/navigation',
        'module-3-ai-brain/sim-to-real'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/index',
        'module-4-vla/vla-paradigm',
        'module-4-vla/voice-action',
        'module-4-vla/llm-planning',
        'module-4-vla/translation'
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid Architecture',
      items: [
        'capstone/index',
        'capstone/end-to-end'
      ],
    },
    {
      type: 'category',
      label: 'Infrastructure',
      items: [
        'infrastructure/index',
        'infrastructure/setup'
      ],
    },
    {
      type: 'doc',
      id: 'conclusion/index',
    },
  ],
};

module.exports = sidebars;
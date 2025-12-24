---
title: End-to-End System Integration
---

# End-to-End System Integration

## Complete System Architecture

The autonomous humanoid robot system integrates all four modules into a cohesive, functional robot that can understand and execute natural language commands. Here's the complete system architecture:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   User Input    │    │  Cognitive Layer │    │  ROS 2 Core     │
│                 │    │                  │    │                 │
│  Voice Command  │───▶│  LLM Planning    │───▶│  Communication  │
│                 │    │  Intent Recog.   │    │  & Control      │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │                          │
                              ▼                          ▼
                    ┌──────────────────┐    ┌─────────────────┐
                    │  Perception &    │    │  Simulation &   │
                    │  Navigation      │◀───┤  Digital Twin   │
                    │                  │    │                 │
                    └──────────────────┘    └─────────────────┘
                              │                          │
                              ▼                          ▼
                    ┌──────────────────┐    ┌─────────────────┐
                    │  Manipulation &  │    │  Humanoid Robot │
                    │  Action Execution│    │  Platform       │
                    └──────────────────┘    └─────────────────┘
```

## Complete System Implementation

Here's the full implementation that brings together all components:

```python
#!/usr/bin/env python3

import rospy
import actionlib
import speech_recognition as sr
import openai
import json
import re
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry

class AutonomousHumanoidSystem:
    def __init__(self):
        rospy.init_node('autonomous_humanoid_system')

        # Initialize all system components
        self.setup_speech_recognition()
        self.setup_llm_planning()
        self.setup_ros_interfaces()
        self.setup_simulation_integration()

        # System state
        self.current_task = None
        self.robot_state = {
            'location': None,
            'battery_level': 100,
            'gripper_status': 'open',
            'current_action': 'idle'
        }

        rospy.loginfo("Autonomous Humanoid System initialized")

    def setup_speech_recognition(self):
        """Setup speech recognition components"""
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # ROS publishers/subscribers for voice system
        self.voice_cmd_pub = rospy.Publisher('/robot/voice_command', String, queue_size=10)
        self.voice_result_pub = rospy.Publisher('/robot/voice_result', String, queue_size=10)

    def setup_llm_planning(self):
        """Setup LLM planning components"""
        # openai.api_key = rospy.get_param('~openai_api_key', 'your-api-key')

        # Publishers/subscribers for planning system
        self.plan_pub = rospy.Publisher('/robot/task_plan', String, queue_size=10)
        self.plan_sub = rospy.Subscriber('/robot/processed_plan', String, self.plan_callback)

    def setup_ros_interfaces(self):
        """Setup all ROS interfaces"""
        # Action clients
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.gripper_client = actionlib.SimpleActionClient('gripper_action', GripperCommandAction)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.speak_pub = rospy.Publisher('/tts_input', String, queue_size=10)
        self.status_pub = rospy.Publisher('/robot/status', String, queue_size=10)

        # Subscribers
        self.command_sub = rospy.Subscriber('/robot/translated_command', String, self.command_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Wait for servers
        rospy.loginfo("Waiting for action servers...")
        self.move_base_client.wait_for_server()
        self.gripper_client.wait_for_server()
        rospy.loginfo("Action servers available")

    def setup_simulation_integration(self):
        """Setup simulation environment integration"""
        # Subscribe to simulation state
        self.sim_state_sub = rospy.Subscriber('/gazebo/model_states', String, self.sim_state_callback)

        # Publishers for simulation interaction
        self.sim_cmd_pub = rospy.Publisher('/gazebo/command', String, queue_size=10)

    def start_system(self):
        """Start the complete autonomous system"""
        rospy.loginfo("Starting autonomous humanoid system...")

        # Start listening for voice commands
        self.start_voice_listening()

        # Keep the node running
        rospy.spin()

    def start_voice_listening(self):
        """Start continuous voice command listening"""
        rospy.loginfo("Voice command system active. Listening...")

        # Use a separate thread for continuous listening
        import threading
        listener_thread = threading.Thread(target=self.continuous_voice_loop)
        listener_thread.daemon = True
        listener_thread.start()

    def continuous_voice_loop(self):
        """Continuous loop for voice command processing"""
        while not rospy.is_shutdown():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=3)

                # Recognize speech
                command_text = self.recognizer.recognize_google(audio)
                rospy.loginfo(f"Heard command: {command_text}")

                # Process the command through the full pipeline
                self.process_full_command(command_text)

            except sr.WaitTimeoutError:
                # Continue listening
                continue
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr(f"Speech recognition error: {e}")
            except Exception as e:
                rospy.logerr(f"Voice processing error: {e}")

    def process_full_command(self, command_text):
        """Process a command through the complete pipeline"""
        try:
            # Step 1: Publish for natural language understanding
            cmd_msg = String()
            cmd_msg.data = command_text
            self.voice_cmd_pub.publish(cmd_msg)

            # Step 2: Generate plan using LLM
            plan = self.generate_llm_plan(command_text)
            if plan:
                # Step 3: Publish plan for execution
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_pub.publish(plan_msg)

                rospy.loginfo(f"Published plan for: {command_text}")
            else:
                rospy.logerr("Failed to generate plan")
                self.speak("I couldn't understand that command.")

        except Exception as e:
            rospy.logerr(f"Error in full command processing: {e}")
            self.speak("Sorry, I encountered an error processing your command.")

    def generate_llm_plan(self, command):
        """Generate a task plan using LLM"""
        # Get current robot state
        state_info = self.get_current_state()

        # Construct the prompt
        prompt = f"""
        You are an autonomous humanoid robot task planner. Given a user command,
        decompose it into a sequence of executable robotic actions.

        Current robot state: {json.dumps(state_info)}

        User command: "{command}"

        Please provide a step-by-step plan in JSON format:
        {{
            "original_command": "{command}",
            "plan": [
                {{"action": "action_name", "parameters": {{"param1": "value1"}}}},
                {{"action": "action_name", "parameters": {{"param1": "value1"}}}}
            ],
            "estimated_duration": "in seconds",
            "confidence": "high/medium/low"
        }}

        Available actions: move_to, navigate_to, pick_up, grasp, speak, wave, wait
        """

        try:
            # Call LLM to generate plan
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.get_planning_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )

            plan_text = response.choices[0].message.content.strip()
            return self.parse_plan(plan_text)

        except Exception as e:
            rospy.logerr(f"LLM planning error: {e}")
            return None

    def get_planning_system_prompt(self):
        """Get the system prompt for LLM planning"""
        return """
        You are an expert autonomous humanoid robot task planner. Generate safe,
        executable plans that consider the robot's current state and environment.

        Guidelines:
        1. Only use actions the robot can perform
        2. Consider current location and battery level
        3. Include error handling where appropriate
        4. Generate clear, sequential steps
        5. Consider safety in all plans
        """

    def parse_plan(self, plan_text):
        """Parse LLM response into structured plan"""
        try:
            # Extract JSON from response
            start = plan_text.find('{')
            end = plan_text.rfind('}') + 1
            if start != -1 and end != 0:
                json_str = plan_text[start:end]
                return json.loads(json_str)
        except json.JSONDecodeError:
            rospy.logwarn("Could not parse LLM response as JSON")

        return None

    def plan_callback(self, msg):
        """Handle received task plan"""
        try:
            plan_data = json.loads(msg.data)
            rospy.loginfo(f"Executing plan: {plan_data['original_command']}")

            # Execute the plan
            success = self.execute_plan(plan_data)

            # Report results
            result = {
                'command': plan_data['original_command'],
                'success': success,
                'timestamp': rospy.Time.now().to_sec()
            }

            result_msg = String()
            result_msg.data = json.dumps(result)
            self.voice_result_pub.publish(result_msg)

            if success:
                self.speak(f"I have completed the task: {plan_data['original_command']}")
            else:
                self.speak(f"I couldn't complete the task: {plan_data['original_command']}")

        except Exception as e:
            rospy.logerr(f"Error executing plan: {e}")

    def execute_plan(self, plan_data):
        """Execute a complete task plan"""
        plan = plan_data.get('plan', [])

        for i, step in enumerate(plan):
            action = step.get('action')
            params = step.get('parameters', {})

            rospy.loginfo(f"Executing step {i+1}/{len(plan)}: {action}")

            success = self.execute_single_action(action, params)

            if not success:
                rospy.logerr(f"Action {action} failed")
                return False

            # Small delay between actions
            rospy.sleep(0.5)

        rospy.loginfo("Plan completed successfully")
        return True

    def execute_single_action(self, action, params):
        """Execute a single action"""
        try:
            if action in ['move_to', 'navigate_to', 'go_to']:
                return self.execute_navigation(params)
            elif action in ['pick_up', 'grasp', 'grab']:
                return self.execute_grasp(params)
            elif action in ['speak', 'say', 'talk']:
                return self.execute_speak(params)
            elif action in ['wave']:
                return self.execute_wave(params)
            elif action in ['wait']:
                return self.execute_wait(params)
            else:
                rospy.logwarn(f"Unknown action: {action}")
                return False
        except Exception as e:
            rospy.logerr(f"Error executing action {action}: {e}")
            return False

    def execute_navigation(self, params):
        """Execute navigation action"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Get target coordinates
        location = params.get('location', 'default')
        target_coords = self.get_location_coordinates(location)

        if target_coords is None:
            rospy.logerr(f"Unknown location: {location}")
            return False

        goal.target_pose.pose.position.x = target_coords[0]
        goal.target_pose.pose.position.y = target_coords[1]
        goal.target_pose.pose.orientation.z = target_coords[2]
        goal.target_pose.pose.orientation.w = target_coords[3]

        self.move_base_client.send_goal(goal)
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(60.0))

        if not finished_within_time:
            rospy.logerr("Navigation timed out")
            return False

        result = self.move_base_client.get_result()
        return result is not None

    def execute_grasp(self, params):
        """Execute grasping action"""
        goal = GripperCommandGoal()
        goal.command.position = params.get('width', 0.0)
        goal.command.max_effort = params.get('effort', 100.0)

        self.gripper_client.send_goal(goal)
        finished_within_time = self.gripper_client.wait_for_result(rospy.Duration(10.0))

        if not finished_within_time:
            rospy.logerr("Grasp action timed out")
            return False

        result = self.gripper_client.get_result()
        return result is not None

    def execute_speak(self, params):
        """Execute speech action"""
        text = params.get('text', '')
        if not text:
            return True

        msg = String()
        msg.data = text
        self.speak_pub.publish(msg)

        # Estimate time based on text length
        text_length = len(text.split())
        rospy.sleep(text_length * 0.2)

        return True

    def execute_wave(self, params):
        """Execute waving action (simplified)"""
        # In a real system, this would control arm joints
        # For simulation, we'll just publish a status
        self.speak("Waving hello!")
        rospy.sleep(2.0)
        return True

    def execute_wait(self, params):
        """Execute wait action"""
        duration = params.get('duration', 1.0)
        rospy.sleep(duration)
        return True

    def get_location_coordinates(self, location_name):
        """Get coordinates for named locations"""
        locations = {
            'kitchen': [2.0, 1.0, 0.0, 1.0],
            'living_room': [0.0, 0.0, 0.0, 1.0],
            'bedroom': [-2.0, 1.0, 0.0, 1.0],
            'office': [1.0, -2.0, 0.0, 1.0],
            'charging_station': [-1.0, -1.0, 0.0, 1.0]
        }
        return locations.get(location_name.lower().replace(' ', '_'))

    def get_current_state(self):
        """Get current robot state"""
        return {
            'location': self.robot_state['location'],
            'battery_level': self.robot_state['battery_level'],
            'gripper_status': self.robot_state['gripper_status'],
            'current_action': self.robot_state['current_action']
        }

    def odom_callback(self, msg):
        """Update robot location from odometry"""
        self.robot_state['location'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.pose.orientation)
        }

    def sim_state_callback(self, msg):
        """Handle simulation state updates"""
        # Process simulation state information
        pass

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        import math
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def speak(self, text):
        """Speak text through TTS"""
        msg = String()
        msg.data = text
        self.speak_pub.publish(msg)

if __name__ == '__main__':
    try:
        system = AutonomousHumanoidSystem()
        system.start_system()
    except rospy.ROSInterruptException:
        rospy.loginfo("System terminated")
```

## System Integration Testing

### Testing the Complete Pipeline

Here's how to test the complete system:

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import json

class SystemTester:
    def __init__(self):
        rospy.init_node('system_tester')

        # Publishers for testing
        self.voice_pub = rospy.Publisher('/robot/voice_command', String, queue_size=10)
        self.status_sub = rospy.Subscriber('/robot/status', String, self.status_callback)

        self.test_results = []

    def run_integration_tests(self):
        """Run complete system integration tests"""
        test_commands = [
            "Go to the kitchen",
            "Move to the living room",
            "Say hello world",
            "Navigate to the bedroom"
        ]

        rospy.loginfo("Starting integration tests...")

        for i, command in enumerate(test_commands):
            rospy.loginfo(f"Running test {i+1}: {command}")
            self.run_single_test(command)
            rospy.sleep(3)  # Wait between tests

        self.report_results()

    def run_single_test(self, command):
        """Run a single integration test"""
        # Publish command
        cmd_msg = String()
        cmd_msg.data = command
        self.voice_pub.publish(cmd_msg)

        # Wait for result
        rospy.sleep(10)  # Wait for execution

    def status_callback(self, msg):
        """Handle system status updates"""
        try:
            status = json.loads(msg.data)
            self.test_results.append(status)
        except:
            pass

    def report_results(self):
        """Report test results"""
        rospy.loginfo("=== INTEGRATION TEST RESULTS ===")
        for i, result in enumerate(self.test_results):
            status = "SUCCESS" if result.get('success', False) else "FAILED"
            rospy.loginfo(f"Test {i+1}: {result.get('command', 'Unknown')} - {status}")

        successful = sum(1 for r in self.test_results if r.get('success', False))
        total = len(self.test_results)
        rospy.loginfo(f"Overall: {successful}/{total} tests passed")

if __name__ == '__main__':
    try:
        tester = SystemTester()
        tester.run_integration_tests()
    except rospy.ROSInterruptException:
        pass
```

## Deployment Configuration

### Launch File for Complete System

Create a launch file to start all components together:

```xml
<launch>
  <!-- Start the autonomous humanoid system -->
  <node name="autonomous_humanoid_system" pkg="your_robot_package" type="autonomous_humanoid_system.py" output="screen">
    <!-- Parameters -->
    <param name="openai_api_key" value="$(arg openai_api_key)" />
  </node>

  <!-- Start navigation stack -->
  <include file="$(find your_robot_navigation)/launch/move_base.launch" />

  <!-- Start simulation (if using Gazebo) -->
  <include file="$(find your_robot_gazebo)/launch/world.launch" />

  <!-- Start speech recognition -->
  <node name="speech_recognition" pkg="your_speech_package" type="speech_node.py" />

  <!-- Start TTS system -->
  <node name="text_to_speech" pkg="your_tts_package" type="tts_node.py" />
</launch>
```

## Performance Optimization

### System Monitoring

Monitor the complete system performance:

```python
class SystemMonitor:
    def __init__(self):
        self.start_time = rospy.Time.now()
        self.action_count = 0
        self.error_count = 0

        # Publishers for monitoring
        self.metrics_pub = rospy.Publisher('/robot/metrics', String, queue_size=10)

    def log_action(self, action, success):
        """Log action execution"""
        self.action_count += 1
        if not success:
            self.error_count += 1

        # Calculate metrics
        uptime = (rospy.Time.now() - self.start_time).to_sec()
        success_rate = (self.action_count - self.error_count) / max(self.action_count, 1)

        metrics = {
            'uptime_seconds': uptime,
            'total_actions': self.action_count,
            'successful_actions': self.action_count - self.error_count,
            'error_count': self.error_count,
            'success_rate': success_rate,
            'timestamp': rospy.Time.now().to_sec()
        }

        # Publish metrics
        msg = String()
        msg.data = json.dumps(metrics)
        self.metrics_pub.publish(msg)
```

## Real-World Deployment Considerations

### Safety and Error Handling

Implement comprehensive safety measures:

1. **Timeout Management**: All actions have appropriate timeouts
2. **Emergency Stop**: Implement emergency stop functionality
3. **Battery Monitoring**: Monitor and manage battery levels
4. **Collision Avoidance**: Integrate collision avoidance systems
5. **User Feedback**: Provide clear feedback about system status

The complete autonomous humanoid system demonstrates the integration of all four modules, creating a robot that can understand natural language commands and execute complex tasks in a simulated environment. This represents the culmination of the Physical AI Robotics book, showing how modern AI techniques can be applied to create truly intelligent robotic systems.
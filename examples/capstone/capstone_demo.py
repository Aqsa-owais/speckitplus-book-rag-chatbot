#!/usr/bin/env python3
"""
Capstone Demo: Autonomous Humanoid Robot System

This example demonstrates the complete autonomous humanoid system
integrating all four modules: ROS 2, Digital Twin, NVIDIA Isaac, and VLA.
"""

import rospy
import actionlib
import speech_recognition as sr
import json
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class CapstoneDemo:
    def __init__(self):
        rospy.init_node('capstone_demo')

        # Initialize all system components
        self.setup_ros_interfaces()
        self.setup_voice_recognition()

        # System state
        self.system_active = True
        self.current_task = None

        rospy.loginfo("=== CAPSTONE DEMO SYSTEM INITIALIZED ===")
        rospy.loginfo("Integrated modules:")
        rospy.loginfo("- ROS 2 Communication Layer")
        rospy.loginfo("- Digital Twin Environment")
        rospy.loginfo("- AI Perception & Navigation")
        rospy.loginfo("- Vision-Language-Action System")

    def setup_ros_interfaces(self):
        """Setup all ROS interfaces for the complete system"""
        # Action clients
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.gripper_client = actionlib.SimpleActionClient('gripper_action', GripperCommandAction)

        # Publishers
        self.status_pub = rospy.Publisher('/robot/status', String, queue_size=10)
        self.speak_pub = rospy.Publisher('/tts_input', String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        self.voice_cmd_sub = rospy.Subscriber('/robot/voice_command', String, self.voice_callback)
        self.task_plan_sub = rospy.Subscriber('/robot/task_plan', String, self.plan_callback)

        # Wait for servers
        rospy.loginfo("Waiting for action servers...")
        if self.move_base_client.wait_for_server(rospy.Duration(10.0)):
            rospy.loginfo("✓ Move base server available")
        else:
            rospy.logwarn("✗ Move base server not available")

        if self.gripper_client.wait_for_server(rospy.Duration(10.0)):
            rospy.loginfo("✓ Gripper server available")
        else:
            rospy.logwarn("✗ Gripper server not available")

    def setup_voice_recognition(self):
        """Setup voice recognition (simplified for demo)"""
        self.voice_commands = [
            "demo start",
            "go to kitchen",
            "pick up object",
            "say hello",
            "wave",
            "return home",
            "demo stop"
        ]

    def voice_callback(self, msg):
        """Handle voice commands"""
        command = msg.data.lower()
        rospy.loginfo(f"Received voice command: {command}")

        if "demo start" in command:
            self.start_demo()
        elif "go to kitchen" in command:
            self.execute_navigation_task("kitchen")
        elif "pick up object" in command:
            self.execute_manipulation_task()
        elif "say hello" in command:
            self.execute_speech_task("Hello, I am your autonomous humanoid assistant!")
        elif "wave" in command:
            self.execute_wave_task()
        elif "return home" in command:
            self.execute_navigation_task("home")
        elif "demo stop" in command:
            self.stop_demo()
        else:
            self.execute_speech_task(f"I heard: {command}, but I'm not sure what to do.")

    def plan_callback(self, msg):
        """Handle task plans from VLA system"""
        try:
            plan_data = json.loads(msg.data)
            rospy.loginfo(f"Executing plan: {plan_data.get('command', 'Unknown')}")
            self.execute_plan(plan_data)
        except json.JSONDecodeError:
            rospy.logerr("Invalid plan format received")

    def start_demo(self):
        """Start the demonstration"""
        rospy.loginfo("=== DEMONSTRATION STARTED ===")
        self.system_active = True
        self.execute_speech_task("Demonstration started. I am now ready to receive commands.")

    def execute_speech_task(self, text):
        """Execute speech task"""
        rospy.loginfo(f"Speaking: {text}")
        msg = String()
        msg.data = text
        self.speak_pub.publish(msg)

    def execute_navigation_task(self, location):
        """Execute navigation task"""
        if not self.move_base_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logerr("Navigation server not available")
            self.execute_speech_task("Sorry, navigation system is not available.")
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Define location coordinates (simplified)
        locations = {
            "kitchen": [2.0, 1.0, 0.0, 1.0],
            "home": [0.0, 0.0, 0.0, 1.0],
            "living_room": [1.0, -1.0, 0.0, 1.0]
        }

        if location in locations:
            coords = locations[location]
            goal.target_pose.pose.position.x = coords[0]
            goal.target_pose.pose.position.y = coords[1]
            goal.target_pose.pose.orientation.z = coords[2]
            goal.target_pose.pose.orientation.w = coords[3]

            rospy.loginfo(f"Navigating to {location}")
            self.move_base_client.send_goal(goal)

            # Wait for result with timeout
            finished = self.move_base_client.wait_for_result(rospy.Duration(30.0))

            if finished:
                rospy.loginfo(f"Successfully reached {location}")
                self.execute_speech_task(f"I have arrived at the {location}.")
                return True
            else:
                rospy.logerr(f"Failed to reach {location}")
                self.execute_speech_task(f"Sorry, I couldn't reach the {location}.")
                return False
        else:
            rospy.logerr(f"Unknown location: {location}")
            self.execute_speech_task(f"Sorry, I don't know where {location} is.")
            return False

    def execute_manipulation_task(self):
        """Execute manipulation task"""
        if not self.gripper_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logerr("Gripper server not available")
            self.execute_speech_task("Sorry, manipulation system is not available.")
            return False

        # Close gripper (simplified)
        goal = GripperCommandGoal()
        goal.command.position = 0.0  # Closed position
        goal.command.max_effort = 50.0

        rospy.loginfo("Executing grasp action")
        self.gripper_client.send_goal(goal)
        finished = self.gripper_client.wait_for_result(rospy.Duration(10.0))

        if finished:
            rospy.loginfo("Grasp completed")
            self.execute_speech_task("I have picked up the object.")
            return True
        else:
            rospy.logerr("Grasp failed")
            self.execute_speech_task("Sorry, I couldn't pick up the object.")
            return False

    def execute_wave_task(self):
        """Execute waving action (simplified)"""
        rospy.loginfo("Waving")
        # In a real system, this would control arm joints
        self.execute_speech_task("Hello! I am waving to you.")
        rospy.sleep(2.0)

    def execute_plan(self, plan_data):
        """Execute a complete task plan"""
        plan = plan_data.get('plan', [])
        original_command = plan_data.get('command', 'Unknown command')

        rospy.loginfo(f"Executing plan for: {original_command}")

        for i, step in enumerate(plan):
            action = step.get('action', '')
            params = step.get('parameters', {})

            rospy.loginfo(f"Step {i+1}/{len(plan)}: {action}")

            success = self.execute_action(action, params)
            if not success:
                rospy.logerr(f"Action failed: {action}")
                self.execute_speech_task(f"Sorry, I couldn't complete the task.")
                return False

            rospy.sleep(0.5)  # Small delay between actions

        rospy.loginfo("Plan completed successfully")
        self.execute_speech_task(f"I have completed the task: {original_command}")
        return True

    def execute_action(self, action, params):
        """Execute a single action"""
        if action == "navigate_to":
            location = params.get('location', 'unknown')
            return self.execute_navigation_task(location)
        elif action == "grasp":
            return self.execute_manipulation_task()
        elif action == "speak":
            text = params.get('text', 'Hello')
            self.execute_speech_task(text)
            return True
        elif action == "wave":
            self.execute_wave_task()
            return True
        else:
            rospy.logwarn(f"Unknown action: {action}")
            return False

    def stop_demo(self):
        """Stop the demonstration"""
        rospy.loginfo("=== DEMONSTRATION STOPPED ===")
        self.system_active = False
        self.execute_speech_task("Demonstration stopped. Thank you for watching!")

    def run_demo_sequence(self):
        """Run a predefined demo sequence"""
        rospy.loginfo("Running demo sequence...")

        # Demo sequence
        sequence = [
            ("speak", {"text": "Starting capstone demonstration"}),
            ("navigate_to", {"location": "kitchen"}),
            ("speak", {"text": "I have reached the kitchen"}),
            ("wave", {}),
            ("speak", {"text": "Thank you for watching the demonstration"})
        ]

        for action, params in sequence:
            rospy.loginfo(f"Executing: {action} with {params}")
            self.execute_action(action, params)
            rospy.sleep(2)

    def start_system(self):
        """Start the complete system"""
        rospy.loginfo("Capstone system ready to receive commands")

        # Option 1: Run predefined demo sequence
        # self.run_demo_sequence()

        # Option 2: Wait for voice commands (will be handled by callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        demo = CapstoneDemo()
        demo.start_system()
    except rospy.ROSInterruptException:
        rospy.loginfo("Capstone demo terminated")
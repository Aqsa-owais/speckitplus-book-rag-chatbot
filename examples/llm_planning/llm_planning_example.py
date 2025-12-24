#!/usr/bin/env python3
"""
Example: LLM-Based Cognitive Planning for Robotics

This example demonstrates how to use Large Language Models for
generating task plans from natural language commands.
"""

import rospy
import openai
import json
from std_msgs.msg import String

class LLMPlanningExample:
    def __init__(self):
        rospy.init_node('llm_planning_example')

        # ROS publishers and subscribers
        self.plan_pub = rospy.Publisher('/robot/task_plan', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/robot/voice_command', String, self.command_callback)

        # Robot capabilities (simplified)
        self.robot_capabilities = {
            "navigation": ["move_to", "navigate_to", "go_to"],
            "manipulation": ["pick_up", "place", "grasp"],
            "communication": ["speak", "wave"]
        }

        rospy.loginfo("LLM planning example initialized")

    def command_callback(self, msg):
        """Process incoming commands and generate plans"""
        command = msg.data
        rospy.loginfo(f"Generating plan for: {command}")

        # Generate plan using LLM
        plan = self.generate_plan(command)
        if plan:
            # Publish the plan
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)
            rospy.loginfo("Plan published successfully")
        else:
            rospy.logerr("Failed to generate plan")

    def generate_plan(self, command):
        """Generate a task plan using LLM"""
        # This is a simplified example - in practice, you'd use a real LLM API
        prompt = f"""
        You are a robotic task planner. Convert the following command to a sequence of robot actions.

        Robot capabilities: {self.robot_capabilities}
        Command: "{command}"

        Provide plan in JSON format:
        {{
            "command": "{command}",
            "plan": [
                {{"action": "action_name", "parameters": {{"param": "value"}}}}
            ]
        }}
        """

        # Simulated LLM response (in practice, call actual LLM API)
        if "go to" in command.lower():
            location = command.lower().replace("go to", "").strip()
            return {
                "command": command,
                "plan": [
                    {"action": "navigate_to", "parameters": {"location": location}}
                ]
            }
        elif "pick up" in command.lower():
            object_name = command.lower().replace("pick up", "").strip()
            return {
                "command": command,
                "plan": [
                    {"action": "navigate_to", "parameters": {"location": "object_location"}},
                    {"action": "grasp", "parameters": {"object": object_name}}
                ]
            }
        elif "say" in command.lower() or "speak" in command.lower():
            text = command.lower().replace("say", "").replace("speak", "").strip()
            return {
                "command": command,
                "plan": [
                    {"action": "speak", "parameters": {"text": text}}
                ]
            }
        else:
            # Default response
            return {
                "command": command,
                "plan": [
                    {"action": "speak", "parameters": {"text": f"I don't know how to {command}"}}
                ]
            }

    def test_planning(self):
        """Test the planning system with sample commands"""
        test_commands = [
            "Go to the kitchen",
            "Pick up the red cup",
            "Say hello world",
            "Navigate to the living room"
        ]

        for command in test_commands:
            rospy.loginfo(f"Testing command: {command}")
            plan = self.generate_plan(command)
            rospy.loginfo(f"Generated plan: {json.dumps(plan, indent=2)}")
            rospy.sleep(2)

if __name__ == '__main__':
    example = LLMPlanningExample()

    # Option 1: Test with sample commands
    # example.test_planning()

    # Option 2: Listen for commands from ROS topic
    rospy.spin()
---
title: LLM-Based Cognitive Planning
---

# LLM-Based Cognitive Planning

## Introduction to LLMs in Robotics

Large Language Models (LLMs) have revolutionized how robots can understand and plan complex tasks. By leveraging the reasoning capabilities of LLMs, robots can interpret high-level natural language commands and decompose them into executable action sequences.

## Role of LLMs in Physical AI

LLMs serve as the cognitive layer in Physical AI systems, bridging the gap between human communication and robotic action. They excel at:

- **Task Decomposition**: Breaking complex goals into smaller, manageable steps
- **Knowledge Integration**: Incorporating world knowledge to inform planning decisions
- **Contextual Reasoning**: Understanding the context of commands and environments
- **Adaptive Planning**: Adjusting plans based on changing conditions or new information

## LLM Integration Architecture

The integration of LLMs with robotic systems typically follows this pattern:

```
Natural Language Command → LLM Processing → Task Plan → ROS 2 Action Execution
```

## Setting Up LLM Integration

### Python Libraries for LLM Integration

Here's how to integrate LLMs with your ROS 2 system:

```python
import rospy
import openai
from std_msgs.msg import String
from std_msgs.msg import Bool
import json
import time

class LLMPlanner:
    def __init__(self):
        rospy.init_node('llm_planner')

        # Initialize OpenAI API (you'll need to set your API key)
        # openai.api_key = rospy.get_param('~openai_api_key', 'your-api-key')

        # ROS publishers and subscribers
        self.plan_pub = rospy.Publisher('/robot/task_plan', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/robot/processed_command', String, self.command_callback)
        self.status_pub = rospy.Publisher('/robot/planner_status', String, queue_size=10)

        # System state
        self.current_environment_state = {}
        self.robot_capabilities = self.get_robot_capabilities()

    def get_robot_capabilities(self):
        """Define what the robot is capable of doing"""
        return {
            "navigation": {
                "actions": ["move_to", "navigate_to", "go_to"],
                "locations": ["kitchen", "living_room", "bedroom", "office"]
            },
            "manipulation": {
                "actions": ["pick_up", "place", "grasp", "release"],
                "objects": ["cup", "book", "bottle", "box"]
            },
            "interaction": {
                "actions": ["wave", "speak", "listen", "wait"]
            }
        }

    def command_callback(self, msg):
        """Process incoming commands and generate plans"""
        command = msg.data
        rospy.loginfo(f"Received command for planning: {command}")

        try:
            plan = self.generate_plan(command)
            if plan:
                self.publish_plan(plan)
                rospy.loginfo(f"Published plan: {plan}")
            else:
                rospy.logerr("Failed to generate plan")
        except Exception as e:
            rospy.logerr(f"Error in planning: {e}")

    def generate_plan(self, command):
        """Generate a task plan using LLM"""
        # Construct the prompt for the LLM
        prompt = self.construct_prompt(command)

        # Call the LLM to generate a plan
        try:
            # Using OpenAI API as an example
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": self.get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,
                max_tokens=500
            )

            plan_text = response.choices[0].message.content.strip()
            return self.parse_plan(plan_text)
        except Exception as e:
            rospy.logerr(f"LLM API error: {e}")
            return None

    def construct_prompt(self, command):
        """Construct the prompt for LLM planning"""
        return f"""
        You are a robotic task planner. Given a user command, decompose it into a sequence of executable robotic actions.

        Environment state: {json.dumps(self.current_environment_state)}
        Robot capabilities: {json.dumps(self.robot_capabilities)}

        User command: "{command}"

        Please provide a step-by-step plan in JSON format with the following structure:
        {{
            "command": "original command",
            "plan": [
                {{"action": "action_name", "parameters": {{"param1": "value1"}}}},
                {{"action": "action_name", "parameters": {{"param1": "value1"}}}}
            ],
            "explanation": "brief explanation of the plan"
        }}

        Actions must be from the robot capabilities list. Be specific and ensure each step is executable.
        """

    def get_system_prompt(self):
        """Get the system prompt for the LLM"""
        return """
        You are an expert robotic task planner. Your role is to decompose high-level natural language commands into executable robotic actions.

        Guidelines:
        1. Only use actions that are within the robot's capabilities
        2. Break down complex tasks into simple, sequential steps
        3. Consider the environment state when planning
        4. If information is ambiguous, make reasonable assumptions based on context
        5. Ensure the plan is feasible and safe
        6. Include error handling steps where appropriate
        """

    def parse_plan(self, plan_text):
        """Parse the LLM response into a structured plan"""
        try:
            # Try to extract JSON from the response
            start = plan_text.find('{')
            end = plan_text.rfind('}') + 1
            if start != -1 and end != 0:
                json_str = plan_text[start:end]
                plan_data = json.loads(json_str)
                return json.dumps(plan_data)
        except json.JSONDecodeError:
            rospy.logwarn("Could not parse LLM response as JSON, attempting to extract plan...")
            # Fallback: try to extract plan information from text
            return self.fallback_parse(plan_text)

        return None

    def fallback_parse(self, plan_text):
        """Fallback parsing if JSON parsing fails"""
        # Simple fallback that creates a basic plan structure
        return json.dumps({
            "command": "fallback",
            "plan": [{"action": "speak", "parameters": {"text": plan_text}}],
            "explanation": "Fallback plan due to parsing error"
        })

    def publish_plan(self, plan_json):
        """Publish the generated plan"""
        plan_msg = String()
        plan_msg.data = plan_json
        self.plan_pub.publish(plan_msg)

    def update_environment_state(self, state_update):
        """Update the current environment state"""
        self.current_environment_state.update(state_update)
```

## Advanced Planning Techniques

### Hierarchical Task Networks (HTN)

LLMs can be used to create hierarchical plans:

```python
class HTNPlanner(LLMPlanner):
    def generate_hierarchical_plan(self, high_level_goal):
        """Generate a hierarchical task network"""
        prompt = f"""
        Decompose the following high-level goal into a hierarchical task network:
        Goal: {high_level_goal}

        Return a plan with high-level tasks that can be further decomposed, in JSON format:
        {{
            "goal": "high-level goal",
            "tasks": [
                {{
                    "name": "task_name",
                    "type": "primitive|compound",
                    "subtasks": [list of subtasks if compound],
                    "prerequisites": [list of prerequisites],
                    "resources": [list of required resources]
                }}
            ]
        }}
        """

        # Call LLM with this prompt to get hierarchical plan
        return self.call_llm(prompt)
```

### Reactive Planning

Combine LLM planning with reactive components:

```python
class ReactiveLLMPlanner(LLMPlanner):
    def __init__(self):
        super().__init__()
        self.current_plan = []
        self.current_step = 0
        self.plan_execution_sub = rospy.Subscriber('/robot/execution_status', String, self.execution_callback)

    def execution_callback(self, msg):
        """Handle execution feedback and potentially replan"""
        status = json.loads(msg.data)

        if status.get('success', False):
            # Move to next step
            self.current_step += 1
            if self.current_step < len(self.current_plan):
                self.execute_next_step()
            else:
                rospy.loginfo("Plan completed successfully")
        else:
            # Handle failure - potentially replan
            self.handle_failure(status)

    def handle_failure(self, status):
        """Handle plan execution failure"""
        error_description = status.get('error', 'Unknown error')
        current_task = self.current_plan[self.current_step] if self.current_step < len(self.current_plan) else None

        # Generate a new plan considering the failure
        replan_prompt = f"""
        The robot failed to execute: {current_task}
        Error: {error_description}
        Current environment state: {json.dumps(self.current_environment_state)}

        Generate a new plan to achieve the original goal, considering the failure.
        """

        new_plan = self.call_llm(replan_prompt)
        if new_plan:
            self.current_plan = new_plan
            self.current_step = 0
            self.execute_next_step()
```

## Safety and Validation

### Plan Validation

Always validate LLM-generated plans before execution:

```python
class SafeLLMPlanner(LLMPlanner):
    def validate_plan(self, plan_json):
        """Validate that the plan is safe and executable"""
        try:
            plan = json.loads(plan_json)

            # Check each action in the plan
            for step in plan.get('plan', []):
                action = step.get('action')

                # Verify action is in robot capabilities
                if not self.is_action_valid(action):
                    rospy.logerr(f"Invalid action in plan: {action}")
                    return False

                # Check parameters
                if not self.validate_parameters(action, step.get('parameters', {})):
                    rospy.logerr(f"Invalid parameters for action {action}")
                    return False

            return True
        except json.JSONDecodeError:
            rospy.logerr("Plan is not valid JSON")
            return False

    def is_action_valid(self, action):
        """Check if an action is valid for this robot"""
        all_actions = []
        for capability in self.robot_capabilities.values():
            all_actions.extend(capability.get('actions', []))
        return action in all_actions

    def validate_parameters(self, action, parameters):
        """Validate action parameters"""
        # Implement parameter validation logic
        # This would check that parameters are of correct type and within valid ranges
        return True  # Simplified for example
```

## Integration with ROS 2 Action Servers

LLM plans can be executed through ROS 2 action servers:

```python
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

class LLMActionExecutor:
    def __init__(self):
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.nav_client.wait_for_server()

    def execute_navigation_task(self, location):
        """Execute a navigation task from LLM plan"""
        # Look up the location in a map (this would be pre-defined)
        pose = self.get_location_pose(location)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        # Send the goal to the navigation system
        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()

        return self.nav_client.get_result()
```

## Best Practices for LLM Integration

1. **Prompt Engineering**: Carefully craft prompts to get consistent, reliable outputs
2. **Validation**: Always validate LLM outputs before execution
3. **Fallback Mechanisms**: Have fallback strategies when LLM responses are inadequate
4. **Caching**: Cache common plans to reduce API calls and improve response time
5. **Monitoring**: Monitor LLM performance and adjust prompts as needed
6. **Safety**: Implement safety checks to prevent dangerous actions

## Considerations for Production Systems

When deploying LLM-based planning in production:

- **Latency**: Consider the time required for LLM API calls
- **Reliability**: Handle API failures gracefully
- **Cost**: Monitor and manage API usage costs
- **Privacy**: Ensure sensitive data isn't sent to external APIs
- **Consistency**: LLM outputs can vary; plan accordingly

LLM-based cognitive planning represents a powerful approach to creating flexible, adaptable robotic systems that can interpret and execute complex natural language commands.
---
title: Voice-to-Action Systems
---

# Voice-to-Action Systems

## Introduction to Voice Commands in Robotics

Voice-to-action systems enable robots to receive and execute commands through natural language spoken by users. This technology bridges the gap between human communication and robotic action, making robots more accessible and intuitive to interact with.

## Architecture of Voice-to-Action Systems

The voice-to-action pipeline consists of several interconnected components:

```
Speech Input → Audio Processing → Speech Recognition → Natural Language Understanding → Action Mapping → ROS 2 Commands
```

## Speech Recognition Setup

### Python Libraries for Speech Recognition

We'll use Python libraries that are compatible with ROS 2 to implement speech recognition:

```python
import speech_recognition as sr
import rospy
from std_msgs.msg import String

class VoiceCommandReceiver:
    def __init__(self):
        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # ROS 2 publisher for commands
        self.command_pub = rospy.Publisher('/robot/command', String, queue_size=10)

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def listen_for_command(self):
        """Listen for and process voice commands"""
        try:
            with self.microphone as source:
                print("Listening for command...")
                audio = self.recognizer.listen(source, timeout=5)

            # Recognize speech using Google's speech recognition
            command_text = self.recognizer.recognize_google(audio)
            print(f"Recognized: {command_text}")

            # Process and publish the command
            self.process_command(command_text)

        except sr.WaitTimeoutError:
            print("No speech detected within timeout")
        except sr.UnknownValueError:
            print("Could not understand audio")
        except sr.RequestError as e:
            print(f"Error with speech recognition service: {e}")

    def process_command(self, command_text):
        """Process the recognized command text"""
        # Publish the recognized command to ROS 2
        cmd_msg = String()
        cmd_msg.data = command_text
        self.command_pub.publish(cmd_msg)
```

### Alternative Speech Recognition Options

Besides Google's API, you can use:

1. **Offline Recognition**:
   - ` pocketsphinx `: Works offline but less accurate
   - ` vosk `: Good offline recognition with downloadable models

2. **Cloud Services**:
   - AWS Transcribe
   - Azure Speech Services
   - Google Cloud Speech-to-Text (more accurate than the basic API)

## Natural Language Understanding

### Intent Recognition

Once speech is converted to text, we need to understand the user's intent:

```python
import re

class IntentRecognizer:
    def __init__(self):
        # Define patterns for different intents
        self.intent_patterns = {
            'navigation': [
                r'go to (.+)',
                r'move to (.+)',
                r'navigate to (.+)',
                r'go (.+)'
            ],
            'object_interaction': [
                r'pick up (.+)',
                r'grasp (.+)',
                r'get (.+)',
                r'take (.+)'
            ],
            'action': [
                r'wave',
                r'stop',
                r'start',
                r'help'
            ]
        }

    def recognize_intent(self, text):
        """Recognize the intent from the text command"""
        text_lower = text.lower()

        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    return {
                        'intent': intent,
                        'parameters': match.groups() if match.groups() else [],
                        'confidence': 0.9  # Simplified confidence
                    }

        return {
            'intent': 'unknown',
            'parameters': [],
            'confidence': 0.0
        }
```

### Entity Extraction

Extract specific objects or locations mentioned in commands:

```python
class EntityExtractor:
    def __init__(self):
        # Define common object and location categories
        self.objects = {
            'furniture': ['table', 'chair', 'desk', 'couch', 'bed'],
            'kitchen': ['cup', 'plate', 'bowl', 'fridge', 'microwave'],
            'rooms': ['kitchen', 'living room', 'bedroom', 'bathroom', 'office']
        }

    def extract_entities(self, text):
        """Extract entities from the text"""
        entities = {}
        text_lower = text.lower()

        for category, items in self.objects.items():
            found_items = []
            for item in items:
                if item in text_lower:
                    found_items.append(item)
            if found_items:
                entities[category] = found_items

        return entities
```

## Voice Command Processing Pipeline

Here's a complete example of how to integrate voice recognition with ROS 2:

```python
#!/usr/bin/env python3

import rospy
import speech_recognition as sr
import re
from std_msgs.msg import String
from geometry_msgs.msg import Pose

class VoiceCommandProcessor:
    def __init__(self):
        rospy.init_node('voice_command_processor')

        # Initialize speech components
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Setup ROS publishers
        self.command_pub = rospy.Publisher('/robot/processed_command', String, queue_size=10)
        self.nav_goal_pub = rospy.Publisher('/move_base_simple/goal', Pose, queue_size=1)

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Start listening loop
        self.start_listening()

    def start_listening(self):
        """Start the continuous listening loop"""
        rospy.loginfo("Voice command processor started. Listening for commands...")

        while not rospy.is_shutdown():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=2)

                # Recognize speech
                command_text = self.recognizer.recognize_google(audio)
                rospy.loginfo(f"Recognized command: {command_text}")

                # Process the command
                self.process_voice_command(command_text)

            except sr.WaitTimeoutError:
                # Continue listening if no command detected
                continue
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr(f"Speech recognition error: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

    def process_voice_command(self, command_text):
        """Process the voice command and execute appropriate action"""
        # Simple command mapping
        if 'go to kitchen' in command_text.lower():
            self.navigate_to_location('kitchen')
        elif 'go to table' in command_text.lower():
            self.navigate_to_location('table')
        elif 'stop' in command_text.lower():
            self.stop_robot()
        elif 'help' in command_text.lower():
            self.provide_help()
        else:
            rospy.logwarn(f"Unknown command: {command_text}")

    def navigate_to_location(self, location):
        """Navigate to a predefined location"""
        # In a real system, this would look up the location in a map
        # For this example, we'll publish a generic navigation command
        cmd_msg = String()
        cmd_msg.data = f"NAVIGATE_TO:{location.upper()}"
        self.command_pub.publish(cmd_msg)

        rospy.loginfo(f"Sending navigation command to {location}")

    def stop_robot(self):
        """Stop the robot's current action"""
        cmd_msg = String()
        cmd_msg.data = "STOP"
        self.command_pub.publish(cmd_msg)

        rospy.loginfo("Sending stop command")

    def provide_help(self):
        """Provide help information"""
        help_text = "Available commands: go to [location], stop, help"
        cmd_msg = String()
        cmd_msg.data = f"HELP:{help_text}"
        self.command_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        processor = VoiceCommandProcessor()
    except rospy.ROSInterruptException:
        pass
```

## Advanced Voice Command Features

### Context-Aware Commands

Implement commands that take into account the robot's current state and environment:

```python
class ContextAwareProcessor:
    def __init__(self):
        self.current_location = None
        self.robot_state = 'idle'
        self.detected_objects = []

    def process_contextual_command(self, command_text, context):
        """Process commands with contextual awareness"""
        # Example: "Go back" - depends on current location
        if 'go back' in command_text.lower():
            return self.navigate_back(context.get('previous_location'))

        # Example: "Go forward" - depends on robot's orientation
        if 'go forward' in command_text.lower():
            return self.move_forward()

        # Example: "Go to the table" - disambiguate based on detected objects
        if 'go to the' in command_text.lower():
            return self.disambiguate_location(command_text, context.get('detected_objects'))
```

### Multi-turn Conversations

Handle complex commands that require multiple interactions:

```python
class ConversationHandler:
    def __init__(self):
        self.conversation_state = {}
        self.pending_requests = {}

    def handle_conversation(self, user_input, session_id):
        """Handle multi-turn conversations"""
        if session_id not in self.conversation_state:
            self.conversation_state[session_id] = {'context': {}, 'step': 0}

        state = self.conversation_state[session_id]

        # Process the current input based on conversation state
        if state['step'] == 0:
            return self.handle_initial_request(user_input, session_id)
        elif state['step'] == 1:
            return self.handle_followup_request(user_input, session_id)
```

## Best Practices for Voice-to-Action Systems

1. **Error Handling**: Always provide clear feedback when commands aren't understood
2. **Confirmation**: For critical actions, ask for confirmation before executing
3. **Fallback**: Provide alternative interaction methods when voice recognition fails
4. **Privacy**: Consider privacy implications of always-listening systems
5. **Robustness**: Handle background noise and different accents effectively

## Integration with VLA Systems

Voice-to-action systems form the input layer of VLA systems, feeding natural language commands that are then processed by the reasoning and action components. The quality of speech recognition directly impacts the overall system's effectiveness.
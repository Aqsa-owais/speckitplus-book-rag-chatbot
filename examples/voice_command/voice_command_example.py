#!/usr/bin/env python3
"""
Example: Voice Command Processing for VLA Systems

This example demonstrates how to implement a voice command receiver
that processes natural language commands and converts them to ROS 2 messages.
"""

import rospy
import speech_recognition as sr
from std_msgs.msg import String

class VoiceCommandExample:
    def __init__(self):
        rospy.init_node('voice_command_example')

        # Initialize speech recognition
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # ROS publishers
        self.command_pub = rospy.Publisher('/robot/voice_command', String, queue_size=10)

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        rospy.loginfo("Voice command example initialized")

    def listen_and_process(self):
        """Continuously listen for voice commands"""
        rospy.loginfo("Listening for voice commands... (Say 'quit' to exit)")

        while not rospy.is_shutdown():
            try:
                with self.microphone as source:
                    rospy.loginfo("Listening...")
                    audio = self.recognizer.listen(source, timeout=5)

                # Recognize speech
                command = self.recognizer.recognize_google(audio)
                rospy.loginfo(f"Heard: {command}")

                # Check for exit command
                if 'quit' in command.lower():
                    rospy.loginfo("Exit command received")
                    break

                # Publish command
                cmd_msg = String()
                cmd_msg.data = command
                self.command_pub.publish(cmd_msg)

            except sr.WaitTimeoutError:
                rospy.loginfo("No speech detected")
                continue
            except sr.UnknownValueError:
                rospy.logwarn("Could not understand audio")
            except sr.RequestError as e:
                rospy.logerr(f"Speech recognition error: {e}")

            rospy.sleep(1)

if __name__ == '__main__':
    example = VoiceCommandExample()
    try:
        example.listen_and_process()
    except rospy.ROSInterruptException:
        pass
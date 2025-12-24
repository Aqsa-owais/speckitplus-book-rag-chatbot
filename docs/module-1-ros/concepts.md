---
sidebar_position: 3
---

# ROS 2 Concepts: Nodes, Topics, Services, and Actions

## Introduction

ROS 2 (Robot Operating System 2) provides a flexible framework for writing robotic software. At its core, ROS 2 is designed to facilitate communication between different software components, called nodes, that run on potentially different machines. Understanding the fundamental communication patterns is essential for building effective robotic systems.

## Nodes

### Definition
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of ROS 2 applications. Each node typically performs a specific task or function within the larger robotic system.

### Key Characteristics
- **Modularity**: Each node performs a specific function, making the system modular and maintainable
- **Distributed**: Nodes can run on different machines but communicate seamlessly
- **Lifecycle**: Nodes can be started, stopped, and managed independently
- **Parameters**: Nodes can accept configuration parameters at runtime

### Example Use Cases
- Sensor driver nodes that publish sensor data
- Control nodes that send commands to actuators
- Perception nodes that process sensor data
- Planning nodes that generate robot trajectories

### Implementation in Python
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
```

## Topics

### Definition
Topics are named buses over which nodes exchange messages. Topics implement a publish-subscribe communication pattern where publishers send messages to a topic and subscribers receive messages from a topic.

### Key Characteristics
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Many-to-many**: Multiple publishers can publish to the same topic; multiple subscribers can subscribe to the same topic
- **Typed**: Each topic has a specific message type that defines the structure of the data
- **Unidirectional**: Data flows in one direction from publisher to subscriber

### Example Use Cases
- Sensor data streams (camera images, LiDAR scans, IMU readings)
- Robot state information (joint positions, odometry)
- Control commands (velocity commands, position targets)

### Implementation in Python
```python
# Publisher
publisher = self.create_publisher(String, 'topic_name', 10)

# Subscriber
subscription = self.create_subscription(
    String,
    'topic_name',
    self.listener_callback,
    10)

def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services

### Definition
Services provide a request-response communication pattern where a client sends a request to a server and waits for a response. Services are synchronous and blocking by default.

### Key Characteristics
- **Synchronous**: Client waits for the server to respond
- **Request-Response**: One request generates one response
- **Bidirectional**: Data flows in both directions (request and response)
- **One-to-one**: One client communicates with one server at a time

### Example Use Cases
- Saving robot configurations
- Triggering specific robot behaviors
- Requesting robot status information
- Calibration procedures

### Implementation in Python
```python
# Service server
service = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    return response

# Service client
client = self.create_client(AddTwoInts, 'add_two_ints')
```

## Actions

### Definition
Actions provide a goal-based communication pattern for long-running tasks. They combine the benefits of both topics and services, allowing for feedback during execution and the ability to cancel ongoing tasks.

### Key Characteristics
- **Goal-Oriented**: Designed for tasks that take a significant amount of time
- **Feedback**: Provides ongoing feedback about task progress
- **Cancelation**: Clients can cancel goals in progress
- **Result**: Returns a final result when the task is completed

### Example Use Cases
- Navigation tasks (moving to a specific location)
- Manipulation tasks (grasping objects)
- Calibration procedures
- Data collection tasks

### Implementation in Python
```python
# Action server
self._action_server = ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    self.execute_callback)

def execute_callback(self, goal_handle):
    feedback_msg = Fibonacci.Feedback()
    feedback_msg.sequence = [0, 1]

    for i in range(1, goal_handle.request.order):
        # Check if there is a cancel request
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return Fibonacci.Result()

        feedback_msg.sequence.append(
            feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1)

    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = feedback_msg.sequence
    return result

# Action client
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
```

## Communication Patterns Comparison

| Pattern | Type | Synchronization | Use Case |
|---------|------|----------------|----------|
| Topics | Publish-Subscribe | Asynchronous | Continuous data streams |
| Services | Request-Response | Synchronous | Single request/response |
| Actions | Goal-Based | Synchronous with feedback | Long-running tasks |

## Best Practices

### Node Design
- Keep nodes focused on a single responsibility
- Use descriptive names for nodes
- Implement proper error handling and logging
- Design nodes to be configurable through parameters

### Topic Usage
- Use appropriate message types for your data
- Consider the frequency of message publishing
- Use quality of service (QoS) settings appropriately
- Follow naming conventions (lowercase with underscores)

### Service Design
- Use services for tasks that have a clear start and end
- Consider the blocking nature when designing your system
- Implement timeouts to prevent indefinite waiting
- Use appropriate service types for your needs

### Action Implementation
- Use actions for tasks that take more than a few seconds
- Provide meaningful feedback during execution
- Handle cancelation requests appropriately
- Return comprehensive results when tasks complete

## Summary

Understanding these fundamental ROS 2 communication patterns is crucial for building effective robotic systems. Each pattern serves different purposes and choosing the right one for your use case is key to creating robust and maintainable robotic applications. In the next section, we'll explore how to implement these concepts using Python and the rclpy library.
---
sidebar_position: 4
---

# Python-based ROS 2 Development using rclpy

## Introduction

The `rclpy` package is the Python client library for ROS 2. It provides a Python API that enables Python programs to communicate with other ROS 2 nodes using the standard ROS 2 communication patterns (topics, services, actions). This guide will walk you through the fundamentals of using rclpy to create ROS 2 nodes in Python.

## Installing rclpy

The `rclpy` package is typically installed as part of the ROS 2 distribution. If you followed the setup guide, it should already be available in your environment. You can verify the installation with:

```bash
python3 -c "import rclpy; print('rclpy is available')"
```

## Basic Node Structure

### Creating a Simple Node

Here's the basic structure of a ROS 2 node using rclpy:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Publishers

### Basic Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher with Custom Message Types

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserPublisher(Node):

    def __init__(self):
        super().__init__('laser_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = -1.57  # -90 degrees in radians
        msg.angle_max = 1.57   # 90 degrees in radians
        msg.angle_increment = 0.01
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.0
        msg.range_max = 10.0

        # Generate some sample data
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = [2.0 + 0.1 * i for i in range(num_readings)]

        self.publisher_.publish(msg)
```

## Working with Subscribers

### Basic Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber with Message Filters

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

        # Create subscribers
        self.image_sub = Subscriber(self, Image, 'image_raw')
        self.info_sub = Subscriber(self, CameraInfo, 'camera_info')

        # Synchronize the topics by time
        ats = ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub],
            queue_size=5,
            slop=0.1
        )
        ats.registerCallback(self.sync_callback)

    def sync_callback(self, image, camera_info):
        self.get_logger().info(f'Got synchronized image and camera info')
        # Process the synchronized data
```

## Working with Services

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Actions

### Creating an Action Server

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile

from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

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

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep to simulate work
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating an Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## Parameters

### Working with Parameters

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('integer_param', 42)
        self.declare_parameter('double_param', 3.14)
        self.declare_parameter('bool_param', True)

        # Get parameter values
        my_param = self.get_parameter('my_parameter').value
        int_param = self.get_parameter('integer_param').value

        self.get_logger().info(f'My parameter: {my_param}')
        self.get_logger().info(f'Integer parameter: {int_param}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

### Error Handling

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

class RobustNode(Node):

    def __init__(self):
        super().__init__('robust_node')

        try:
            self.declare_parameter('config_param', 'default')
            config_value = self.get_parameter('config_param').value
        except ParameterNotDeclaredException:
            self.get_logger().error('Parameter not declared')
            config_value = 'default'
```

### Logging

```python
import rclpy
from rclpy.node import Node

class LoggingNode(Node):

    def __init__(self):
        super().__init__('logging_node')

        # Different log levels
        self.get_logger().debug('Debug message')
        self.get_logger().info('Info message')
        self.get_logger().warn('Warning message')
        self.get_logger().error('Error message')
        self.get_logger().fatal('Fatal message')
```

### Resource Management

```python
import rclpy
from rclpy.node import Node

class ResourceNode(Node):

    def __init__(self):
        super().__init__('resource_node')

        # Create timers, publishers, subscribers
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pub = self.create_publisher(String, 'topic', 10)

    def destroy_node(self):
        # Clean up resources before destroying the node
        self.timer.cancel()
        super().destroy_node()
```

## Summary

The rclpy library provides a comprehensive Python interface to ROS 2 functionality. By understanding how to create nodes, publishers, subscribers, services, and actions using rclpy, you can build sophisticated robotic applications in Python. The examples provided here cover the most common use cases, but rclpy supports many additional features including parameter handling, logging, and quality of service settings that you can explore as needed for your specific applications.
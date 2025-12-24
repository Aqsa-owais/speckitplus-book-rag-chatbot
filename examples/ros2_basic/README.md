# Simple ROS 2 Publisher/Subscriber Example

This example demonstrates the fundamental ROS 2 communication pattern using a publisher and subscriber.

## Overview

The example consists of two nodes:
- `publisher_member_function.py`: Publishes "Hello World" messages to a topic
- `subscriber_member_function.py`: Subscribes to the topic and logs received messages

## How It Works

### Publisher Node
1. Creates a publisher that sends `String` messages to the topic named `topic`
2. Uses a timer to publish a message every 0.5 seconds
3. Increments a counter with each message to show sequence

### Subscriber Node
1. Creates a subscriber that listens to the `topic` topic
2. Uses a callback function to process incoming messages
3. Logs the received message content to the console

## Running the Example

1. **Terminal 1 - Start the publisher:**
```bash
python3 publisher_member_function.py
```

2. **Terminal 2 - Start the subscriber:**
```bash
python3 subscriber_member_function.py
```

You should see the publisher sending messages and the subscriber receiving them:

Publisher output:
```
[INFO] [1612345678.123456789] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1612345678.623456789] [minimal_publisher]: Publishing: "Hello World: 1"
```

Subscriber output:
```
[INFO] [1612345678.123456789] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [1612345678.623456789] [minimal_subscriber]: I heard: "Hello World: 1"
```

## Key Concepts Demonstrated

- **Node Creation**: How to create ROS 2 nodes using classes
- **Publisher**: How to create and use publishers to send messages
- **Subscriber**: How to create and use subscribers to receive messages
- **Message Types**: Using standard message types like `std_msgs/String`
- **Timer**: Using timers to publish messages at regular intervals
- **Logging**: Using the built-in logging system

## Code Structure

### Publisher Code
```python
# Create publisher
self.publisher_ = self.create_publisher(String, 'topic', 10)

# Create timer to publish periodically
self.timer = self.create_timer(timer_period, self.timer_callback)

# Publish message
msg = String()
msg.data = 'Hello World: %d' % self.i
self.publisher_.publish(msg)
```

### Subscriber Code
```python
# Create subscriber
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)

# Callback function
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

This example demonstrates the publish-subscribe communication pattern that is fundamental to ROS 2 and enables different parts of a robotic system to communicate with each other.
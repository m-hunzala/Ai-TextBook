# Physical AI ROS 2 Nodes

This package contains simple ROS 2 nodes for the Physical AI chapter. These nodes demonstrate basic publisher-subscriber communication in ROS 2 using Python (rclpy).

## Nodes

### simple_talker
- A publisher node that sends "Hello from Physical AI chapter" messages to the `chatter` topic every second
- Demonstrates basic ROS 2 publisher implementation

### simple_listener  
- A subscriber node that listens to the `chatter` topic
- Logs received messages to the console

## Prerequisites

- ROS 2 (Humble Hawksbill or later)
- Python 3.8 or later
- Standard ROS 2 Python packages

## Installation

1. Clone this repository or copy the nodes directory to your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   # Copy the physical_ai_nodes package here
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select physical_ai_nodes
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

1. Run the talker node:
   ```bash
   ros2 run physical_ai_nodes simple_talker
   ```

2. In a separate terminal, run the listener node:
   ```bash
   ros2 run physical_ai_nodes simple_listener
   ```

3. You should see the talker publishing messages and the listener receiving them.

## Topics

- `chatter` (std_msgs/String): The topic used for communication between the nodes

## Implementation Details

The nodes follow ROS 2 best practices:
- Proper node initialization and cleanup
- Timer-based publishing
- Asynchronous message handling
- Proper logging
- Exception handling for clean shutdown

This example demonstrates the basic building blocks for more complex ROS 2 applications in physical AI and robotics.
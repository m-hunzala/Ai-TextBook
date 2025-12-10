# Claude Code Subagents Documentation

## Overview

The system implements reusable intelligence through Claude Code Subagents that can be invoked to handle specialized tasks. These subagents are designed to transform user requests into code snippets, action plans, or specialized responses.

## Available Subagents

### 1. ROS Helper (ros-helper)
**Description**: A specialist in ROS2 (Robot Operating System 2) that can parse ROS2 examples, generate launch files, nodes, and suggest appropriate ROS2 commands. Handles topics, services, actions, packages, and common ROS2 patterns.

**Capabilities**:
- Parse and generate ROS2 Python and C++ code
- Create launch files and configuration
- Suggest ROS2 commands for debugging and running
- Generate package.xml and setup.py files
- Handle ROS2 concepts like nodes, topics, services, and actions

**Sample Interactions**:

**User Query**: "How do I create a ROS2 publisher in Python?"

**Subagent Response**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Simulation Helper (sim-helper)
**Description**: A specialist in robotics simulation using Gazebo, Ignition, and related tools. Can suggest simulation commands, create world files, configure simulation parameters, and provide advice on simulation best practices.

**Capabilities**:
- Create Gazebo world files in SDF format
- Suggest simulation commands for launching and controlling simulations
- Configure robot models and environments
- Provide physics engine recommendations
- Generate sensor configurations

**Sample Interactions**:

**User Query**: "Create a Gazebo world with a ground plane and a cube"

**Subagent Response**:
```xml
<sdf version="1.7">
  <world name="simple_world">
    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Simple Box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## System Integration

### Automatic Detection
The RAG system automatically detects when a user query should be handled by a specialized subagent:
- Queries containing "ros", "ros2", "rclpy", "rclcpp", "colcon", etc. trigger the ROS Helper
- Queries containing "gazebo", "ignition", "simulation", "sdf", "urdf", etc. trigger the Simulation Helper

### Manual Invocation
Users can also manually invoke subagents using the "Use Expert Skill" button in the chat interface.

## API Endpoints

### GET /subagents
Returns a list of available subagents with their descriptions.

### POST /subagent-call
Calls a specific subagent with a query.

Request body:
```json
{
  "subagent_name": "ros-helper",
  "query": "Create a ROS2 publisher node",
  "context": {}
}
```

Response:
```json
{
  "result": {
    "summary": "ROS2 publisher node implementation",
    "code_snippets": [
      {
        "language": "python",
        "title": "Minimal Publisher Node",
        "code": "# Python code here",
        "description": "Implementation of a ROS2 publisher"
      }
    ],
    "commands": ["ros2 run my_package publisher_node"],
    "explanation": "Detailed explanation of the code"
  }
}
```

## UI Integration

The chat widget includes:
- A "Use Expert Skill" button that shows available subagents
- Special formatting for subagent results with code blocks and explanations
- Automatic detection of subagent-appropriate queries
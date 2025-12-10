# roscore-subagent

This subagent generates ROS 2 package skeletons with rclpy templates to quickly bootstrap new ROS 2 nodes.

## Input Schema

```json
{
  "type": "object",
  "properties": {
    "package_name": {
      "type": "string",
      "description": "Name of the ROS 2 package to create",
      "minLength": 1,
      "maxLength": 50
    },
    "nodes": {
      "type": "array",
      "description": "List of nodes to create in the package",
      "items": {
        "type": "object",
        "properties": {
          "name": {
            "type": "string",
            "description": "Name of the node",
            "minLength": 1
          },
          "description": {
            "type": "string",
            "description": "Brief description of the node's purpose"
          },
          "publishers": {
            "type": "array",
            "description": "List of topics this node will publish",
            "items": {
              "type": "object",
              "properties": {
                "topic_name": {
                  "type": "string",
                  "description": "Name of the topic to publish"
                },
                "message_type": {
                  "type": "string",
                  "description": "ROS 2 message type for the topic"
                }
              },
              "required": ["topic_name", "message_type"]
            }
          },
          "subscribers": {
            "type": "array",
            "description": "List of topics this node will subscribe to",
            "items": {
              "type": "object",
              "properties": {
                "topic_name": {
                  "type": "string",
                  "description": "Name of the topic to subscribe"
                },
                "message_type": {
                  "type": "string",
                  "description": "ROS 2 message type for the topic"
                }
              },
              "required": ["topic_name", "message_type"]
            }
          }
        },
        "required": ["name", "description"]
      }
    },
    "dependencies": {
      "type": "array",
      "description": "Additional ROS 2 dependencies for the package",
      "items": {
        "type": "string"
      }
    }
  },
  "required": ["package_name", "nodes"]
}
```

## Output Schema

```json
{
  "type": "object",
  "properties": {
    "package_path": {
      "type": "string",
      "description": "Path where the package was created"
    },
    "files_created": {
      "type": "array",
      "description": "List of files created in the package",
      "items": {
        "type": "string"
      }
    },
    "instructions": {
      "type": "string",
      "description": "Instructions for building and running the package"
    }
  },
  "required": ["package_path", "files_created", "instructions"]
}
```

## Example Invocation

```json
{
  "package_name": "my_robot_controller",
  "nodes": [
    {
      "name": "motor_controller",
      "description": "Controls the robot's motors based on velocity commands",
      "publishers": [
        {
          "topic_name": "/motor/status",
          "message_type": "std_msgs/String"
        }
      ],
      "subscribers": [
        {
          "topic_name": "/cmd_vel",
          "message_type": "geometry_msgs/Twist"
        }
      ]
    },
    {
      "name": "sensor_processor",
      "description": "Processes sensor data from robot's sensors",
      "publishers": [
        {
          "topic_name": "/processed_sensors",
          "message_type": "sensor_msgs/PointCloud2"
        }
      ],
      "subscribers": [
        {
          "topic_name": "/raw_sensors",
          "message_type": "sensor_msgs/LaserScan"
        }
      ]
    }
  ],
  "dependencies": [
    "geometry_msgs",
    "sensor_msgs"
  ]
}
```

## Agent Skill Description

The **roscore-subagent** is a powerful tool for rapidly generating ROS 2 package skeletons with complete rclpy node templates. This agent creates well-structured ROS 2 packages with proper directory structure, setup files, and functional node templates that include publishers, subscribers, and other ROS 2 communication patterns. The generated code follows ROS 2 best practices and includes documentation for further development. This tool significantly reduces the setup time for new ROS 2 projects by providing working templates that can be immediately built and executed.
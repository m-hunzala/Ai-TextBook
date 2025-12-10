# isaac-sim-subagent

This subagent generates Python scripts to load USD robots in Isaac Sim and provides a simple navigation example for the robot.

## Input Schema

```json
{
  "type": "object",
  "properties": {
    "robot_name": {
      "type": "string",
      "description": "Name of the robot to load in Isaac Sim",
      "minLength": 1,
      "maxLength": 50
    },
    "robot_usd_path": {
      "type": "string",
      "description": "Path to the robot's USD file"
    },
    "world_usd_path": {
      "type": "string",
      "description": "Path to the world USD file (optional, defaults to simple room)",
      "default": "/Isaac/Environments/Simple_Room/simple_room.usd"
    },
    "initial_position": {
      "type": "object",
      "description": "Initial position of the robot in the world",
      "properties": {
        "x": {
          "type": "number",
          "description": "X coordinate in meters",
          "default": 0.0
        },
        "y": {
          "type": "number",
          "description": "Y coordinate in meters",
          "default": 0.0
        },
        "z": {
          "type": "number",
          "description": "Z coordinate in meters",
          "default": 0.0
        }
      }
    },
    "robot_joints": {
      "type": "array",
      "description": "List of joint names for the robot",
      "items": {
        "type": "string"
      }
    },
    "sensor_configurations": {
      "type": "array",
      "description": "List of sensors to attach to the robot",
      "items": {
        "type": "object",
        "properties": {
          "sensor_type": {
            "type": "string",
            "description": "Type of sensor to attach",
            "enum": ["camera", "lidar", "imu", "depth_camera", "rtx_lidar"]
          },
          "sensor_name": {
            "type": "string",
            "description": "Name for the sensor"
          },
          "position": {
            "type": "object",
            "properties": {
              "x": {"type": "number"},
              "y": {"type": "number"},
              "z": {"type": "number"}
            },
            "required": ["x", "y", "z"]
          },
          "orientation": {
            "type": "object",
            "properties": {
              "x": {"type": "number"},
              "y": {"type": "number"},
              "z": {"type": "number"},
              "w": {"type": "number"}
            },
            "required": ["x", "y", "z", "w"]
          }
        },
        "required": ["sensor_type", "sensor_name", "position", "orientation"]
      }
    },
    "navigation_example": {
      "type": "object",
      "description": "Configuration for the navigation example",
      "properties": {
        "algorithm": {
          "type": "string",
          "description": "Navigation algorithm to use",
          "enum": ["simple_navigation", "path_following", "waypoint_navigation"],
          "default": "simple_navigation"
        },
        "waypoints": {
          "type": "array",
          "description": "List of waypoints for navigation",
          "items": {
            "type": "object",
            "properties": {
              "x": {"type": "number"},
              "y": {"type": "number"},
              "z": {"type": "number"}
            },
            "required": ["x", "y", "z"]
          }
        },
        "max_velocity": {
          "type": "number",
          "description": "Maximum velocity for navigation",
          "default": 1.0,
          "minimum": 0.001
        },
        "max_angular_velocity": {
          "type": "number",
          "description": "Maximum angular velocity for navigation",
          "default": 1.0,
          "minimum": 0.001
        }
      }
    },
    "ros_bridge_enabled": {
      "type": "boolean",
      "description": "Whether to enable ROS bridge",
      "default": false
    },
    "output_directory": {
      "type": "string",
      "description": "Directory to save generated files",
      "default": "./isaac_sim_examples"
    }
  },
  "required": ["robot_name", "robot_usd_path"]
}
```

## Output Schema

```json
{
  "type": "object",
  "properties": {
    "python_script_content": {
      "type": "string",
      "description": "Generated Python script for Isaac Sim"
    },
    "python_script_path": {
      "type": "string",
      "description": "Path where the Python script was saved"
    },
    "config_files_created": {
      "type": "array",
      "description": "List of additional config files created",
      "items": {
        "type": "string"
      }
    },
    "instructions": {
      "type": "string",
      "description": "Instructions for running the Isaac Sim example"
    },
    "dependencies": {
      "type": "array",
      "description": "List of required dependencies for the script",
      "items": {
        "type": "string"
      }
    }
  },
  "required": ["python_script_content", "python_script_path", "instructions", "dependencies"]
}
```

## Example Invocation

```json
{
  "robot_name": "my_robot",
  "robot_usd_path": "/path/to/my_robot.usd",
  "world_usd_path": "/Isaac/Environments/Simple_Room/simple_room.usd",
  "initial_position": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "robot_joints": [
    "joint1",
    "joint2",
    "joint3"
  ],
  "sensor_configurations": [
    {
      "sensor_type": "camera",
      "sensor_name": "rgb_camera",
      "position": {
        "x": 0.1,
        "y": 0.0,
        "z": 0.1
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    },
    {
      "sensor_type": "lidar",
      "sensor_name": "lidar_sensor",
      "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.2
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    }
  ],
  "navigation_example": {
    "algorithm": "waypoint_navigation",
    "waypoints": [
      {
        "x": 1.0,
        "y": 0.0,
        "z": 0.0
      },
      {
        "x": 1.0,
        "y": 1.0,
        "z": 0.0
      },
      {
        "x": 0.0,
        "y": 1.0,
        "z": 0.0
      }
    ],
    "max_velocity": 0.5,
    "max_angular_velocity": 0.5
  },
  "ros_bridge_enabled": true,
  "output_directory": "./isaac_sim_examples"
}
```

## Agent Skill Description

The **isaac-sim-subagent** creates Python scripts to load and control robots in NVIDIA Isaac Sim, a high-fidelity simulation environment. This agent generates complete examples that include robot loading, sensor configuration, and navigation algorithms. The generated code handles USD file loading, physics properties, sensor integration, and various navigation strategies. Additionally, the agent can enable ROS bridge for communication between Isaac Sim and ROS-based nodes. This tool simplifies the process of setting up sophisticated simulation environments with realistic physics and rendering for advanced robotics research and development.
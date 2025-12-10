# gazebo-launcher-subagent

This subagent generates Gazebo launch files and SDF world files for robot simulation in Gazebo Classic or Gazebo Garden.

## Input Schema

```json
{
  "type": "object",
  "properties": {
    "robot_name": {
      "type": "string",
      "description": "Name of the robot to spawn in Gazebo",
      "minLength": 1,
      "maxLength": 50
    },
    "robot_urdf_path": {
      "type": "string",
      "description": "Path to the robot's URDF file"
    },
    "world_name": {
      "type": "string",
      "description": "Name of the Gazebo world to create",
      "minLength": 1,
      "maxLength": 50
    },
    "spawn_position": {
      "type": "object",
      "description": "Position to spawn the robot in the world",
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
        },
        "roll": {
          "type": "number",
          "description": "Roll rotation in radians",
          "default": 0.0
        },
        "pitch": {
          "type": "number",
          "description": "Pitch rotation in radians",
          "default": 0.0
        },
        "yaw": {
          "type": "number",
          "description": "Yaw rotation in radians",
          "default": 0.0
        }
      }
    },
    "physics_engine": {
      "type": "string",
      "description": "Physics engine to use",
      "enum": ["ode", "bullet", "dart"],
      "default": "ode"
    },
    "world_properties": {
      "type": "object",
      "description": "Properties for the Gazebo world",
      "properties": {
        "gravity": {
          "type": "array",
          "description": "Gravity vector [x, y, z] in m/s^2",
          "items": {
            "type": "number"
          },
          "minItems": 3,
          "maxItems": 3,
          "default": [0.0, 0.0, -9.8]
        },
        "ground_plane": {
          "type": "boolean",
          "description": "Whether to include a ground plane",
          "default": true
        },
        "obstacles": {
          "type": "array",
          "description": "List of static obstacles in the world",
          "items": {
            "type": "object",
            "properties": {
              "name": {
                "type": "string",
                "description": "Name of the obstacle"
              },
              "type": {
                "type": "string",
                "description": "Type of obstacle: 'box', 'cylinder', 'sphere'",
                "enum": ["box", "cylinder", "sphere"]
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
              "dimensions": {
                "type": "object",
                "properties": {
                  "width": {"type": "number", "description": "Width for box obstacles"},
                  "height": {"type": "number", "description": "Height for box/cylinder obstacles"},
                  "radius": {"type": "number", "description": "Radius for sphere/cylinder obstacles"}
                }
              },
              "mass": {
                "type": "number",
                "description": "Mass of the obstacle in kg",
                "minimum": 0.001
              }
            },
            "required": ["name", "type", "position", "dimensions"]
          }
        }
      }
    },
    "plugins": {
      "type": "array",
      "description": "List of Gazebo plugins to include",
      "items": {
        "type": "string",
        "enum": [
          "joint_state_publisher",
          "robot_state_publisher",
          "gazebo_ros_control",
          "camera_sensor",
          "lidar_sensor",
          "imu_sensor"
        ]
      }
    },
    "ros_bridge": {
      "type": "boolean",
      "description": "Whether to include ROS bridge for communication",
      "default": true
    }
  },
  "required": ["robot_name", "robot_urdf_path", "world_name"]
}
```

## Output Schema

```json
{
  "type": "object",
  "properties": {
    "launch_file_content": {
      "type": "string",
      "description": "Generated launch file content (XML)"
    },
    "launch_file_path": {
      "type": "string",
      "description": "Path where the launch file was saved"
    },
    "sdf_world_content": {
      "type": "string",
      "description": "Generated SDF world file content"
    },
    "sdf_world_path": {
      "type": "string",
      "description": "Path where the SDF world file was saved"
    },
    "mesh_files_needed": {
      "type": "array",
      "description": "List of mesh files needed for the simulation",
      "items": {
        "type": "string"
      }
    },
    "instructions": {
      "type": "string",
      "description": "Instructions for running the simulation"
    }
  },
  "required": ["launch_file_content", "launch_file_path", "sdf_world_content", "sdf_world_path", "instructions"]
}
```

## Example Invocation

```json
{
  "robot_name": "my_differential_robot",
  "robot_urdf_path": "package://my_robot_description/urdf/my_robot.urdf",
  "world_name": "simple_room",
  "spawn_position": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  },
  "physics_engine": "ode",
  "world_properties": {
    "gravity": [0.0, 0.0, -9.8],
    "ground_plane": true,
    "obstacles": [
      {
        "name": "table1",
        "type": "box",
        "position": {
          "x": 2.0,
          "y": 1.0,
          "z": 0.5
        },
        "dimensions": {
          "width": 1.0,
          "height": 0.8,
          "radius": 0.5
        },
        "mass": 10.0
      },
      {
        "name": "cylinder1",
        "type": "cylinder",
        "position": {
          "x": -1.0,
          "y": -1.0,
          "z": 0.25
        },
        "dimensions": {
          "height": 0.5,
          "radius": 0.2
        },
        "mass": 5.0
      }
    ]
  },
  "plugins": [
    "joint_state_publisher",
    "robot_state_publisher",
    "gazebo_ros_control",
    "lidar_sensor"
  ],
  "ros_bridge": true
}
```

## Agent Skill Description

The **gazebo-launcher-subagent** creates complete Gazebo simulation environments with launch files and world descriptions. This agent generates all necessary files to run physics-based simulations of robots in realistic environments. It handles the configuration of physics engines, sensor plugins, robot spawning positions, and world elements including obstacles and terrain. The generated files follow ROS 2 best practices and include proper ROS bridges for communication between the simulation and ROS nodes. This tool streamlines the setup process for robot simulation and testing in Gazebo environments.
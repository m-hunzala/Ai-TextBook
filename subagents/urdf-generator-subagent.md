# urdf-generator-subagent

This subagent generates minimal URDF files for robots based on provided specifications including masses, link lengths, joint limits, and mesh placeholders.

## Input Schema

```json
{
  "type": "object",
  "properties": {
    "robot_name": {
      "type": "string",
      "description": "Name of the robot",
      "minLength": 1,
      "maxLength": 50
    },
    "links": {
      "type": "array",
      "description": "List of links in the robot",
      "items": {
        "type": "object",
        "properties": {
          "name": {
            "type": "string",
            "description": "Name of the link",
            "minLength": 1
          },
          "mass": {
            "type": "number",
            "description": "Mass of the link in kg",
            "minimum": 0.001
          },
          "length": {
            "type": "number",
            "description": "Length of the link in meters",
            "minimum": 0.001
          },
          "radius": {
            "type": "number",
            "description": "Radius of the link in meters (for cylindrical links)",
            "minimum": 0.001
          },
          "width": {
            "type": "number",
            "description": "Width of the link in meters (for box links)",
            "minimum": 0.001
          },
          "height": {
            "type": "number",
            "description": "Height of the link in meters (for box links)",
            "minimum": 0.001
          },
          "mesh_file": {
            "type": "string",
            "description": "Path to mesh file for the link (optional)"
          },
          "color": {
            "type": "string",
            "description": "Color for the link in URDF visual (e.g., 'Red', 'Blue')",
            "default": "Grey"
          }
        },
        "required": ["name", "mass", "length"]
      }
    },
    "joints": {
      "type": "array",
      "description": "List of joints connecting the links",
      "items": {
        "type": "object",
        "properties": {
          "name": {
            "type": "string",
            "description": "Name of the joint",
            "minLength": 1
          },
          "type": {
            "type": "string",
            "description": "Type of joint: 'revolute', 'continuous', 'prismatic', 'fixed'",
            "enum": ["revolute", "continuous", "prismatic", "fixed"]
          },
          "parent": {
            "type": "string",
            "description": "Parent link name"
          },
          "child": {
            "type": "string",
            "description": "Child link name"
          },
          "origin_xyz": {
            "type": "array",
            "description": "Origin position [x, y, z] in meters",
            "items": {
              "type": "number"
            },
            "minItems": 3,
            "maxItems": 3
          },
          "origin_rpy": {
            "type": "array",
            "description": "Origin rotation [roll, pitch, yaw] in radians",
            "items": {
              "type": "number"
            },
            "minItems": 3,
            "maxItems": 3
          },
          "axis_xyz": {
            "type": "array",
            "description": "Joint axis [x, y, z] in meters",
            "items": {
              "type": "number"
            },
            "minItems": 3,
            "maxItems": 3
          },
          "lower_limit": {
            "type": "number",
            "description": "Lower limit for revolute joints in radians"
          },
          "upper_limit": {
            "type": "number",
            "description": "Upper limit for revolute joints in radians"
          },
          "effort_limit": {
            "type": "number",
            "description": "Effort limit for the joint in N or N-m",
            "default": 100.0
          },
          "velocity_limit": {
            "type": "number",
            "description": "Velocity limit for the joint in rad/s or m/s",
            "default": 1.0
          }
        },
        "required": ["name", "type", "parent", "child", "origin_xyz", "axis_xyz"]
      }
    }
  },
  "required": ["robot_name", "links", "joints"]
}
```

## Output Schema

```json
{
  "type": "object",
  "properties": {
    "urdf_content": {
      "type": "string",
      "description": "Generated URDF XML content"
    },
    "urdf_file_path": {
      "type": "string",
      "description": "Path where the URDF file was saved"
    },
    "mesh_files_created": {
      "type": "array",
      "description": "List of mesh files created as placeholders",
      "items": {
        "type": "string"
      }
    },
    "validation_status": {
      "type": "string",
      "description": "Status of URDF validation"
    }
  },
  "required": ["urdf_content", "urdf_file_path", "validation_status"]
}
```

## Example Invocation

```json
{
  "robot_name": "simple_arm",
  "links": [
    {
      "name": "base_link",
      "mass": 1.0,
      "length": 0.2,
      "radius": 0.1,
      "color": "Grey"
    },
    {
      "name": "link1",
      "mass": 0.5,
      "length": 0.3,
      "radius": 0.05,
      "color": "Red"
    },
    {
      "name": "link2",
      "mass": 0.3,
      "length": 0.25,
      "radius": 0.04,
      "color": "Blue"
    },
    {
      "name": "end_effector",
      "mass": 0.2,
      "length": 0.1,
      "radius": 0.03,
      "color": "Green"
    }
  ],
  "joints": [
    {
      "name": "joint_base_to_link1",
      "type": "revolute",
      "parent": "base_link",
      "child": "link1",
      "origin_xyz": [0.0, 0.0, 0.2],
      "origin_rpy": [0.0, 0.0, 0.0],
      "axis_xyz": [0.0, 0.0, 1.0],
      "lower_limit": -1.57,
      "upper_limit": 1.57,
      "effort_limit": 100.0,
      "velocity_limit": 1.0
    },
    {
      "name": "joint_link1_to_link2",
      "type": "revolute",
      "parent": "link1",
      "child": "link2",
      "origin_xyz": [0.0, 0.0, 0.3],
      "origin_rpy": [0.0, 0.0, 0.0],
      "axis_xyz": [0.0, 1.0, 0.0],
      "lower_limit": -1.57,
      "upper_limit": 1.57,
      "effort_limit": 100.0,
      "velocity_limit": 1.0
    },
    {
      "name": "joint_link2_to_end",
      "type": "revolute",
      "parent": "link2",
      "child": "end_effector",
      "origin_xyz": [0.0, 0.0, 0.25],
      "origin_rpy": [0.0, 0.0, 0.0],
      "axis_xyz": [0.0, 1.0, 0.0],
      "lower_limit": -1.57,
      "upper_limit": 1.57,
      "effort_limit": 50.0,
      "velocity_limit": 1.0
    }
  ]
}
```

## Agent Skill Description

The **urdf-generator-subagent** creates complete URDF (Unified Robot Description Format) files for robots based on provided specifications. This tool automatically generates the kinematic structure, dynamic properties, and visual representations with proper joint constraints and limits. The agent creates placeholder mesh files when actual mesh paths are not provided, making it easy to visualize and simulate the robot in Gazebo or RViz. The generated URDF follows ROS standards and includes proper inertial properties, making it ready for physics simulation and control algorithm development.
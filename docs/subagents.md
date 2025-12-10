---
sidebar_position: 15
---

# Claude Code Subagents for Robotics

## Overview

Claude Code Subagents are specialized tools that can generate ROS 2 code, URDF files, simulation environments, and grade student assignments automatically. These subagents accelerate development and education by handling routine tasks with consistent, well-structured outputs.

## Available Subagents

### 1. roscore-subagent

Generates ROS 2 package skeletons with rclpy templates to quickly bootstrap new ROS 2 nodes.

**Use Cases:**
- Rapid prototyping of new ROS 2 packages
- Creating template nodes with publishers/subscribers
- Setting up project structure following ROS 2 best practices

[View roscore-subagent Documentation](subagents/roscore-subagent.md)

### 2. urdf-generator-subagent

Generates minimal URDF files for robots based on provided specifications including masses, link lengths, joint limits, and mesh placeholders.

**Use Cases:**
- Creating robot descriptions for simulation
- Preparing URDF files for Gazebo or RViz visualization
- Generating robot models with proper kinematic chains

[View urdf-generator-subagent Documentation](subagents/urdf-generator-subagent.md)

### 3. gazebo-launcher-subagent

Generates Gazebo launch files and SDF world files for robot simulation in Gazebo Classic or Gazebo Garden.

**Use Cases:**
- Setting up physics-based simulation environments
- Creating world files with obstacles and terrain
- Configuring sensor plugins and ROS bridges

[View gazebo-launcher-subagent Documentation](subagents/gazebo-launcher-subagent.md)

### 4. isaac-sim-subagent

Generates Python scripts to load USD robots in Isaac Sim and provides a simple navigation example for the robot.

**Use Cases:**
- Setting up high-fidelity simulation in Isaac Sim
- Configuring sensors and navigation algorithms
- Creating realistic environments for advanced robotics research

[View isaac-sim-subagent Documentation](subagents/isaac-sim-subagent.md)

### 5. assignment-grader-subagent

Takes a student repository link, runs tests, verifies ROS topics, and returns detailed rubric scores.

**Use Cases:**
- Automated grading of ROS-based assignments
- Consistent evaluation against detailed rubrics
- Providing detailed feedback to students

[View assignment-grader-subagent Documentation](subagents/assignment-grader-subagent.md)

## Benefits of Using Subagents

### For Developers
- **Speed**: Generate boilerplate code and configuration files in seconds
- **Consistency**: All outputs follow ROS 2 best practices and standards
- **Reliability**: Well-tested templates that work correctly from the start

### For Educators
- **Automation**: Grade assignments quickly and consistently
- **Detailed Feedback**: Students receive specific feedback on their code
- **Scalability**: Handle large class sizes with consistent evaluation

### For Students
- **Learning Aids**: Understand proper ROS 2 project structure
- **Starting Points**: Get working templates to build from
- **Best Practices**: Learn proper coding patterns from well-structured examples

## Getting Started

To use any of these subagents, provide the required input parameters in JSON format. Each subagent will generate the appropriate files and provide instructions for use. The generated code follows ROS 2 standards and includes proper documentation and comments to help you understand and modify the code as needed.

Each subagent documentation page contains:
- Complete input schema specification
- Output schema with all generated elements
- Example invocations with real use cases
- Detailed skill descriptions for implementation guidance
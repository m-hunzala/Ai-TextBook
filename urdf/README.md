# Simple Humanoid URDF

This URDF (Unified Robot Description Format) file defines a simple humanoid robot for use in robotics simulations. The robot includes a basic body structure with arms and legs suitable for educational purposes.

## Robot Structure

The humanoid robot consists of the following links and joints:

### Links (Body Parts):
- **base_link**: The main base of the robot
- **torso**: Main body section
- **head**: Robot head with spherical shape
- **left_upper_arm / right_upper_arm**: Upper arm sections
- **left_lower_arm / right_lower_arm**: Lower arm sections
- **left_upper_leg / right_upper_leg**: Upper leg sections
- **left_lower_leg / right_lower_leg**: Lower leg sections

### Joints:
- Fixed joints connect the torso, legs, and head
- Revolute joints allow arm movement at the shoulders and elbows
- Gazebo plugin for joint state publishing

## Usage

### In Gazebo:
```bash
roslaunch gazebo_ros empty_world.launch
rosrun gazebo_ros spawn_model -file $(rospack find your_package)/urdf/simple_humanoid.urdf -urdf -model simple_humanoid
```

### In RViz:
```bash
roslaunch robot_state_publisher robot_state_publisher.launch
rosrun rviz rviz
```

## Design Features

- **Mass Properties**: Properly defined mass and inertia values for physics simulation
- **Visual Elements**: Simple geometric shapes with distinct colors for each body part
- **Collision Geometry**: Matching collision models for each visual element
- **Joint Limits**: Proper limits defined for movable joints
- **Gazebo Integration**: Includes necessary plugins for simulation

## Physical Properties

- Total approximate mass: ~15 kg
- Base dimensions: 0.2 x 0.2 x 0.1 meters
- Torso dimensions: 0.15 x 0.15 x 0.3 meters
- Arm segments: Various sizes as appropriate for humanoid proportions

## Extending the Model

This basic humanoid can be extended by:
- Adding more joints for increased mobility
- Including sensors (cameras, IMUs, etc.)
- Adding actuators for more sophisticated control
- Including more detailed mesh files for realistic appearance

## Educational Use

This model is designed for educational purposes to demonstrate:
- URDF best practices
- Robot kinematics
- Simulation concepts
- ROS integration with robot models
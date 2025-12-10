---
sidebar_position: 14
---

# Translation Feature Example

This is an example page demonstrating the translation feature. You can translate this content to Urdu using the button below.

## Sample Technical Content

Here's some technical content to demonstrate how the translation preserves code and technical terms:

```python
import rospy
from sensor_msgs.msg import LaserScan

def lidar_callback(data):
    # Process LIDAR data
    ranges = data.ranges
    print(f"LIDAR readings: {len(ranges)} points")
```

This function processes LIDAR sensor data from a robot. The `rospy` library is used for ROS communication, and `LaserScan` is the message type for LIDAR data.

## How to Use Translation

1. Click the "اردو میں ترجمہ کریں" button at the top of this page
2. Wait for the translation to complete (this may take a few seconds)
3. View the translated content with preserved code blocks

## More Technical Examples

The robot uses inverse kinematics to calculate joint angles:

```urdf
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

The URDF file defines the robot's structure with a `base_link` and visual geometry. Both the XML tags and the file format remain unchanged during translation.

## Additional Context

This chapter covers the fundamentals of robotics programming. The `sensor_msgs` package provides message definitions for various sensors. The `ranges` field contains distance measurements from the laser scanner.

```bash
# Install required packages
sudo apt-get install ros-noetic-sensor-msgs
```

The bash command installs the ROS sensor messages package. The package name and command remain unchanged during translation.
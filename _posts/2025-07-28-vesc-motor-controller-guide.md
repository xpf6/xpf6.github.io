---
title: "VESC Motor Controller Guide for ROS2"
excerpt: "A comprehensive guide to using VESC motor controllers with ROS2, including bug fixes, configuration, and implementation examples."
categories:
  - Robotics
  - ROS2
tags:
  - vesc
  - motor-controller
  - ros2
  - ackermann
  - robotics
toc: true
font_size: small
toc_label: "Contents"
toc_icon: "cog"
lang: en
languages:
  - name: "English"
    code: "en"
    url: "/robotics/ros2/vesc-motor-controller-guide/"
  - name: "中文"
    code: "zh"
    url: "/robotics/ros2/vesc-motor-controller-guide-zh/"
date: 2024-01-15
last_modified_at: 2024-01-15
author_profile: true
---

{% include language-switcher.html %}

VESC (Vedder Electronic Speed Controller) is a powerful open-source motor controller widely used in robotics applications. This guide covers integration with ROS2, common issues, and practical implementation examples.

<img src="/assets/images/vesc.jpg" alt="VESC Motor Controller" class="embedded-image">

## Bug Fix: ROS2 Package Issue

The `vesc_ackermann` package contains a bug in the `vesc_to_odom` node where motor direction reversal doesn't affect encoder output, causing negative odometry when moving forward.

### Quick Solution

Simply negate the `speed_to_erpm_gain_` parameter in your YAML configuration:

```yaml
# In your config file
speed_to_erpm_gain_: -4614.0  # Use negative value
```

**Note:** This is simpler than modifying source code and doesn't require recompilation.
{: .notice--info}

## Hardware Setup

### BLDC Motor Control
VESC provides multiple control modes: duty cycle, current, brake, speed, and position control.

### Servo Motor Control
Enable servo output in VESC Tool: `App Settings → General → Enable Servo Output = True`

## ROS2 Integration

### Package Installation

```bash
# Clone and build
git clone -b ros2 https://github.com/f1tenth/vesc
cd vesc
git reset --hard 153998df8545fe1781b975df88e411b4e71d4bfe
colcon build
```

**Troubleshooting:** Add `find_package(serial_driver REQUIRED)` to `vesc_driver/CMakeLists.txt` if build fails.
{: .notice--warning}

### Package Overview

#### vesc_driver
- **Publishes:** `sensors/core`, `sensors/servo_position_command`, `sensors/imu`
- **Subscribes:** `commands/motor/speed`, `commands/servo/position`, etc.

#### ackermann_to_vesc
- **Purpose:** Converts Ackermann commands to VESC commands
- **Key Parameters:** `speed_to_erpm_gain`, `steering_angle_to_servo_gain`

#### vesc_to_odom  
- **Purpose:** Generates odometry from VESC sensor data
- **Key Parameters:** `wheelbase`, `odom_frame`, `base_frame`

## Basic Usage

### Start VESC Driver
```bash
ros2 run vesc_driver vesc_driver_node
```

### Control Commands

**Servo Control:**
```bash
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.5"
```

**Motor Speed Control:**
```bash
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 1500.0"
```

**Motor Duty Cycle:**
```bash
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.2"
```

## Custom Implementation

### cmd_vel to VESC Converter

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math

class CmdVelToVesc(Node):
    def __init__(self):
        super().__init__('cmdvel_to_vesc')
        
        # Parameters
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('speed_to_erpm_gain', 4614.0)
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.speed_gain = self.get_parameter('speed_to_erpm_gain').value
        
        # Publishers
        self.motor_pub = self.create_publisher(Float64, 'commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, 'commands/servo/position', 10)
        
        # Subscriber
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
    
    def cmd_callback(self, msg):
        # Convert to motor speed
        motor_speed = msg.linear.x * self.speed_gain
        
        # Convert to servo position
        if abs(msg.linear.x) > 0.01:
            steering_angle = math.atan(msg.angular.z * self.wheelbase / msg.linear.x)
            servo_pos = 0.5 + (steering_angle / math.pi) * 0.5
        else:
            servo_pos = 0.5
            
        # Publish commands
        self.motor_pub.publish(Float64(data=motor_speed))
        self.servo_pub.publish(Float64(data=max(0.0, min(1.0, servo_pos))))

def main():
    rclpy.init()
    node = CmdVelToVesc()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

Common issues and solutions:

1. **Negative Odometry**: Use negative `speed_to_erpm_gain_` value
2. **Build Errors**: Add missing dependencies to CMakeLists.txt
3. **Servo Range**: Default range is 0.15-0.85, can extend to 0.0-1.0

## References and Resources

- [VESC Project](https://vesc-project.com/): Official website and documentation
- [F1TENTH VESC](https://github.com/f1tenth/vesc): ROS2 package repository
- [ROS2 Transport Drivers](https://github.com/ros-drivers/transport_drivers/tree/humble): Additional drivers

## Video Tutorials

- [VESC Setup Guide](https://youtu.be/aPldHJLQ7j8?si=5gIQ9VwuY8PUZZJT): Complete setup walkthrough

---

*This guide covers essential VESC integration with ROS2. For advanced configurations, consult the official documentation.*
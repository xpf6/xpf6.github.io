---
title: "VESC电机控制器ROS2使用指南"
excerpt: "详细介绍如何在ROS2中使用VESC电机控制器，包括问题修复、配置和实现示例。"
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
toc_label: "目录"
toc_icon: "cog"
lang: zh
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

VESC（Vedder电子调速器）是一款强大的开源电机控制器，在机器人应用中被广泛使用。本指南涵盖了与ROS2的集成、常见问题和实际实现示例。

<img src="/assets/images/vesc.jpg" alt="VESC电机控制器" class="embedded-image">

## Bug修复：ROS2包问题

`vesc_ackermann`包的`vesc_to_odom`节点中存在一个bug，电机方向反转不会影响编码器输出，导致向前移动时里程计为负值。

### 快速解决方案

在YAML配置文件中对`speed_to_erpm_gain_`参数取负值：

```yaml
# 在配置文件中
speed_to_erpm_gain_: -4614.0  # 使用负值
```

**注意：** 此方法比修改源代码更简单，且不需要重新编译。
{: .notice--info}

## 硬件设置

### BLDC电机控制
VESC提供多种控制模式：占空比、电流、刹车、速度和位置控制。

### 伺服电机控制
在VESC Tool中启用伺服输出：`App Settings → General → Enable Servo Output = True`

## ROS2集成

### 包安装

```bash
# 克隆并构建
git clone -b ros2 https://github.com/f1tenth/vesc
cd vesc
git reset --hard 153998df8545fe1781b975df88e411b4e71d4bfe
colcon build
```

**故障排除：** 如果构建失败，在`vesc_driver/CMakeLists.txt`中添加`find_package(serial_driver REQUIRED)`。
{: .notice--warning}

### 包概览

#### vesc_driver
- **发布话题：** `sensors/core`、`sensors/servo_position_command`、`sensors/imu`
- **订阅话题：** `commands/motor/speed`、`commands/servo/position`等

#### ackermann_to_vesc
- **功能：** 将阿克曼命令转换为VESC命令
- **关键参数：** `speed_to_erpm_gain`、`steering_angle_to_servo_gain`

#### vesc_to_odom
- **功能：** 从VESC传感器数据生成里程计
- **关键参数：** `wheelbase`、`odom_frame`、`base_frame`

## 基本使用

### 启动VESC驱动
```bash
ros2 run vesc_driver vesc_driver_node
```

### 控制命令

**伺服控制：**
```bash
ros2 topic pub /commands/servo/position std_msgs/msg/Float64 "data: 0.5"
```

**电机速度控制：**
```bash
ros2 topic pub /commands/motor/speed std_msgs/msg/Float64 "data: 1500.0"
```

**电机占空比：**
```bash
ros2 topic pub /commands/motor/duty_cycle std_msgs/msg/Float64 "data: 0.2"
```

## 自定义实现

### cmd_vel到VESC转换器

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
        
        # 参数设置
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('speed_to_erpm_gain', 4614.0)
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.speed_gain = self.get_parameter('speed_to_erpm_gain').value
        
        # 发布器
        self.motor_pub = self.create_publisher(Float64, 'commands/motor/speed', 10)
        self.servo_pub = self.create_publisher(Float64, 'commands/servo/position', 10)
        
        # 订阅器
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
    
    def cmd_callback(self, msg):
        # 转换为电机速度
        motor_speed = msg.linear.x * self.speed_gain
        
        # 转换为伺服位置
        if abs(msg.linear.x) > 0.01:
            steering_angle = math.atan(msg.angular.z * self.wheelbase / msg.linear.x)
            servo_pos = 0.5 + (steering_angle / math.pi) * 0.5
        else:
            servo_pos = 0.5
            
        # 发布命令
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

## 故障排除

常见问题及解决方案：

1. **里程计为负值**：使用负的`speed_to_erpm_gain_`值
2. **构建错误**：在CMakeLists.txt中添加缺失的依赖项
3. **伺服范围**：默认范围是0.15-0.85，可扩展到0.0-1.0

## 参考资料

- [VESC项目](https://vesc-project.com/)：官方网站和文档
- [F1TENTH VESC](https://github.com/f1tenth/vesc)：ROS2包仓库
- [ROS2传输驱动器](https://github.com/ros-drivers/transport_drivers/tree/humble)：附加驱动程序

## 视频教程

- [VESC设置指南](https://youtu.be/aPldHJLQ7j8?si=5gIQ9VwuY8PUZZJT)：完整设置演练

---

*本指南涵盖了VESC与ROS2集成的核心内容。如需高级配置，请参考官方文档。*
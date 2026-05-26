# robot_hardware_interface

ROS 2 Hardware Interface (HI) for a **6-DOF robot arm** with serial communication to an Arduino.

## Overview

| Feature | Details |
|---|---|
| ROS 2 version | Humble / Iron / Jazzy |
| Hardware interface type | `SystemInterface` |
| Command interfaces | `position` (6 joints, radians) |
| State interfaces | `position` (mirrors commands — no encoders) |
| Controller | `joint_trajectory_controller` |
| Transport | Serial (POSIX termios) |

## Serial Protocol

Each control cycle the HI sends **one line** to the Arduino:

```
J <j0_deg> <j1_deg> <j2_deg> <j3_deg> <j4_deg> <j5_deg>\n
```

Angles are in **degrees** (2 decimal places). The Arduino is responsible for
driving the servos / steppers.

On activation the HI sends `HOME\n` so the robot starts at a known position.

### Minimal Arduino sketch

```cpp
void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "HOME") {
      // Move all joints to 0°
      return;
    }

    if (line.startsWith("J ")) {
      float angles[6];
      sscanf(line.c_str(), "J %f %f %f %f %f %f",
             &angles[0], &angles[1], &angles[2],
             &angles[3], &angles[4], &angles[5]);

      // TODO: drive your servos/steppers to angles[]
    }
  }
}
```

## Package layout

```
robot_hardware_interface/
├── CMakeLists.txt
├── package.xml
├── plugin.xml
├── include/robot_hardware_interface/
│   └── hardware_interface.hpp
├── src/
│   └── hardware_interface.cpp
└── urdf/
    └── ros2_control.urdf.xacro   ← example snippet for your URDF
```

## URDF integration

Copy the `<ros2_control>` block from `urdf/ros2_control.urdf.xacro` into your
robot description. Adjust joint names and limits to match your model.

```xml
<hardware>
  <plugin>robot_hardware_interface/RobotHardwareInterface</plugin>
  <param name="serial_port">/dev/ttyACM0</param>
  <param name="baud_rate">115200</param>
</hardware>
```

## Build

```bash
# From the root of your ROS 2 workspace:
colcon build --packages-select robot_hardware_interface
source install/setup.bash
```

## Serial port permissions

```bash
sudo usermod -aG dialout $USER
# Log out and back in, then verify:
ls -l /dev/ttyUSB0
```

## Controller configuration (controller_manager)

```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    command_interfaces:
      - position
    state_interfaces:
      - position
    open_loop_control: true   # recommended when there are no encoders
    allow_partial_joints_goal: false
```

Launch with:

```bash
ros2 launch robot_bringup robot.launch.py
```

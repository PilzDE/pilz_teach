# Teleoperation
Teleoperation (jogging) for a manipulator e.g. using a keyboard interface.

## Getting Started
Bring up your robot and jog-arm-server; see for example [prbt_jog_arm package](../prbt_jog_arm_support/README.md).

To start the keyboard frontend:

```
rosrun pilz_teleoperation key_teleop.py
```

To move the robot around, use keypad numbers as described in the on-screen instructions.

## Future Plans
The driver is extensible to any other input device, so that you can interface
a teach panel and jog your robot with ROS.

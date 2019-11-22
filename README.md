# pilz_teach

pilz_teach is a PRBT6 teaching software module:
*Jogging* the manipulator and *defining* poses as you are used to in robotics controllers, but using native ROS functionality.

It can be used without additional hardware (keyboard-only-mode by default) but has easy interfaces to connect to any external device.


## Package `prbt_jog_arm_support`
[prbt_jog_arm_support](prbt_jog_arm_support/README.md) contains launch and config files to bring up the robot together with jog_arm.

## Package `pilz_teleoperation`
[pilz_teleoperation](pilz_teleoperation/README.md) contains a python driver and frondend to jog the robot interactively.

## Related project links
* [pilz_robots](http://wiki.ros.org/pilz_robots) package and tutorials for getting started with PRBT6 robot manipulator module
* [jog_arm, currently in moveit_experimental](https://github.com/ros-planning/moveit) used for jogging the manipulator
  best used together with
  [tork-a/fake_joint](https://github.com/tork-a/fake_joint) if testing jogging in a simulation environment.
* TODO: evaluate [tork-a/jog_control](https://github.com/tork-a/jog_control)?!
* [ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard/),
  see [#16](https://github.com/ros-teleop/teleop_twist_keyboard/pull/16) there for 6D-extension
* [ros-teleop/teleop_tools](https://github.com/ros-teleop/teleop_tools)

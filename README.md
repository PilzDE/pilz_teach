# Disclaimer

At the moment we are considering, if it is possible yet to develop this package for ROS2.

Until now, it is not possible to use ros1_bridge to bridge actions from ROS to ROS2 and vice versa. Thus we are not able to send a MotionPlanRequest to MoveIt. There is an [issue](https://github.com/ros2/design/issues/195) at the ROS2 design targetting this matter. However, there doesn't even seem to exist an accepted design proposal for bridging actions, so we can't expect an anon implementation of this functionality.

[action_bridge](https://github.com/ipa-hsd/action_bridge) can send action goals from ROS to ROS2 only. Since we need to transmit in the other direction, this is of no use for us yet.

We are able to send JointStates, so it may be possible to implement a ROS node, which
- subscribes to a topic that accepts JointState messages
- builds MotionPlanningRequests from received JointStates
- has a client which sends them to the respective MoveIt Action Server

TODO: We may create an issue at ros1_bridge (concerning the need for bridging actions) and furthermore ask for a feature at [action_bridge](https://github.com/ipa-hsd/action_bridge) for further development ROS2->ROS.

# pilz_teach

pilz_teach is a PRBT6 teaching software module:
*Jogging* the manipulator and *defining* poses as you are used to in robotics controllers, but using native ROS functionality.

It can be used without additional hardware (keyboard-only-mode by default) but has easy interfaces to connect to any external device.

## Instructions

### Prerequisites

- Install ros-melodic-desktop from binaries, following the [official instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Install the pilz packages by executing `apt install ros-melodic-pilz-*` and `apt install ros-melodic-prbt-*`
- Install ros-melodic-desktop from binaries, following the [official instructions](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)
- Create a separate workspace (here called `dashing_ws`) and put [ros1_bridge](https://github.com/ros2/ros1_bridge/tree/dashing
), [moveit_msgs](https://github.com/ros-planning/moveit_msgs/tree/ros2), [object_recognition_msgs](https://github.com/AcutronicRobotics/object_recognition_msgs) and [octomap_msgs](https://github.com/AcutronicRobotics/octomap_msgs) into its src folder

### Build Dashing and the Bridge

- Make sure that you didn't set any ros-related environment variables for ROS or ROS2 in your current bash and go to your ros2_dashing workspace
- Go to your `dashing_ws`
- Then source your melodic installation and your dashing installation (`source /opt/ros/dashing/setup.bash`)
- Build the messages by executing `colcon build --symlink-install --packages-skip ros1_bridge`
- Source the build: `source install/setup.bash`
- Build the ros1_bridge by executing `colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure`
- Source again build: `source install/setup.bash`

Congratulations - you can now run the bridge via `ros2 run ros1_bridge dynamic_bridge`. Be default, only msg and srv are mapped (and thus visible), which exist in both, ROS and ROS2. You can change that behaviour by running the bridge with one of the flags
- `--bridge-all-1to2-topics`
- `--bridge-all-2to1-topics`
- `--bridge-all-topics`

### Usage example

For this, you need three separate bashes

In the first bash:
- Source your melodic environment
- Execute `roslaunch prbt_moveit_config moveit_planning_execution.launch -p 11311`

In the second bash:
- Source your melodic environment and your dashing environment
- Make sure the ROS master is known with `export ROS_MASTER_URI=http://localhost:11311`
- Execute `ros2 run ros1_bridge dynamic_bridge --bridge-all-topics`

In the third bash:
- Source your dashing environment
- Publish to the /joint_states topic, e.g. using `ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: ['prbt_joint_1']
position: [1]
velocity: [0]
effort: [0]"`

Now the manipulator depicted in RViz should move back and forth.

### ROS2 Eloquent

The beforementioned procedure can analoguously be done with ROS2 Eloquent without any changes.

## Related project links
* [pilz_robots](http://wiki.ros.org/pilz_robots) package and tutorials for getting started with PRBT6 robot manipulator module
* [jog_arm, currently in moveit_experimental](https://github.com/ros-planning/moveit) used for jogging the manipulator
  best used together with
  [tork-a/fake_joint](https://github.com/tork-a/fake_joint) if testing jogging in a simulation environment.
* TODO: evaluate [tork-a/jog_control](https://github.com/tork-a/jog_control)?!
* [ros-teleop/teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard/),
  see [#16](https://github.com/ros-teleop/teleop_twist_keyboard/pull/16) there for 6D-extension
* [ros-teleop/teleop_tools](https://github.com/ros-teleop/teleop_tools)

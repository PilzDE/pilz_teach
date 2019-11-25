## Disclaimer

Since the ros1_bridge doesn't provide bridging custom topics by now, when installed via binaries, we decided to stop developing this package for ROS2 and wait for a distribution, which supports straightforward bridging of custom messages.# pilz_teach

pilz_teach is a PRBT6 teaching software module:
*Jogging* the manipulator and *defining* poses as you are used to in robotics controllers, but using native ROS functionality.

It can be used without additional hardware (keyboard-only-mode by default) but has easy interfaces to connect to any external device.

## Instructions

### Prerequisites

- Install ros-melodic-desktop from binaries, following the [official instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Install the pilz packages by executing `apt install ros-melodic-pilz-*` and `apt install ros-melodic-prbt-*`
- Follow the [instructions for building ROS2 dashing from source](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/), but don't build it yet
- Create a separate workspace (here called it `dashing_ws`) and put [moveit_msgs](https://github.com/ros-planning/moveit_msgs/tree/ros2), [object_recognition_msgs](https://github.com/AcutronicRobotics/object_recognition_msgs) and [octomap_msgs](https://github.com/AcutronicRobotics/octomap_msgs) into its src folder

### Build Dashing and the Bridge

- Make sure that you didn't set any ros-related environment variables for ROS or ROS2 in your current bash and go to your ros2_dashing workspace
- Build ROS2 save the ros1_bridge by executing `colcon build --symlink-install --packages-skip ros1_bridge`
- Then source your melodic installation, your dashing installation (`source ~/ros2_dashing/install/local_setup.bash`) and every workspace which contains message definitions you want be mapped from ROS to ROS2 and vice versa
- Build the ros1_bridge by executing `colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure`

Congratulations - you can now run the bridge via `ros2 run ros1_bridge dynamic_bridge`. Be default, only msg and srv are mapped (and thus visible), which exist in both, ROS and ROS2. You can change that behaviour by running the bridge with on of the flags
- `--bridge-all-1to2-topics`
- `--bridge-all-2to1-topics`
- `--bridge-all-topics`

### Build the moveit message definitions

- Go to your `dashing_ws`
- Execute `colcon build` to build the messages
- Execute `source ~/dashing_ws/install/local_setup.bash` to source your environment

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

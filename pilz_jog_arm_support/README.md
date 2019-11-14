# Jog Arm Support
This Package contains files to launch the prbt with moveit_jog_arm and tork-a/fake_joint in simulation mode

## Perquisites
* [pilz_robots](http://wiki.ros.org/pilz_robots)
* [jog_arm, currently in moveit_experimental](https://github.com/ros-planning/moveit)
* [tork-a/fake_joint](https://github.com/tork-a/fake_joint)

## Installation
- Install the pilz packages from debian packages.

   ```apt install ros-melodic-pilz-robots```
- Install moveit packages from source ...
  - following the instructions [here](https://moveit.ros.org/install/source/)
  - or with the packages ```apt install ros-melodic-moveit```
- Get the [tork-a/fake_joint](https://github.com/tork-a/fake_joint) driver from the github and build it.


## Quick Start
To test the setup execute following steps:
1. Start the prbt: 
```roslaunch pilz_jog_arm_support prbt_jog_arm_sim.launch```

2. Move the robot out of the singularity (e.g. with the Motion Planner Plugin of rviz).

3. Change controllers: 
```shell script
rosservice call /controller_manager/switch_controller "start_controllers:
 ['joint_group_position_controller']
stop_controllers:
 ['manipulator_joint_trajectory_controller']
strictness: 2"
```

4. Start moveit_jog_arm: 
```shell script
roslaunch pilz_jog_arm_support jog_server.launch
```
5. Send test command:
```shell script
rostopic pub -r 100 /jog_server/delta_jog_cmds geometry_msgs/TwistStamped "header: auto
twist:
  linear:
    x: 0.0
    y: 0.1
    z: -0.1
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
```

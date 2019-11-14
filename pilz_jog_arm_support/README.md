# Jog Arm Support
This Package contains files to launch the prbt with moveit_jog_arm and tork-a/fake_joint in simulation mode.

## Prerequisites
- You need a workstation or [Virtual machine](https://en.wikipedia.org/wiki/Virtual_machine) with [Ubuntu 18.04 LTS](http://releases.ubuntu.com/releases/18.04/) and [ROS melodic](http://wiki.ros.org/melodic) installed.
- Create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

## Installation from source
- clone this package into the src directory `git clone https://github.com/PilzDE/pilz_teach` 

- Install moveit packages from source ...
  - following the [moveit documentation](https://moveit.ros.org/install/source/)
  - or with the packages `apt install ros-melodic-moveit` (currently jog_arm is not yet available as binary!)

- install all the dependencies `rosdep install -y --from-paths src --ignore-src`

- get tork-a/fake_joint for simulation `git clone https://github.com/tork-a/fake_joint`

- build e.g. in `~/catkin_ws` using `catkin build` and `source devel/setup.bash`

## Quick Start
To test the setup execute following steps:

1. Startup the prbt: 
```
roslaunch pilz_jog_arm_support prbt_jog_arm_sim.launch
```
2. Send test command:
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


## Related Packages
* [pilz_robots](http://wiki.ros.org/pilz_robots)
* [jog_arm, currently in moveit_experimental](https://github.com/ros-planning/moveit)
* [tork-a/fake_joint](https://github.com/tork-a/fake_joint)


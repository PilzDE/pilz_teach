# Teleoperation
Teleoperation (jogging) for a manipulator e.g. using a keyboard interface.

## Keyboard Teleoperation

### Getting Started
Bring up your robot and jog-arm-server; see for example [prbt_jog_arm package](../prbt_jog_arm_support/README.md).

To start the keyboard frontend:

```
rosrun pilz_teleoperation key_teleop.py
```

To move the robot around, use keypad numbers as described in the on-screen instructions.

### Alternative Key Bindings

To use alternative keyboard bindings for the key_teleop driver do:
- create a separate config yaml file.
Use following Syntax:
    ```
    MovementBindings:
      'w': <Twist> (e.g. '8': {linear: {x: 1.0, y: -1.0}})
    JointJogBindings:
      '<CURSES_KEY_NAME>': {"joint_names": ["prbt_joint_2", "prbt_joint_3"], "velocities": [.3, -.1]}
    SettingBindings:
      '+': <SettingName> (e.g. '+': "INCREASE_LINEAR_VELOCITY")
    Description: |-
      Displayed Text e.g. to show current bindings
      With the '|-' the text can contain multiple rows
    ```
    
    See [SetTeleopSettings.srv](./srv/SetTeleopSettings.srv) for available settings.

    See [Curses key names](https://www.gnu.org/software/guile-ncurses/manual/html_node/Getting-characters-from-the-keyboard.html)
    for name of special characters in curses.

    See [Default Keyboard Configuration](./config/keyboard_binding.yaml) for reference.

- then start the node with the additional argument "_bindings"

    ```
    rosrun pilz_teleoperation key_teleop.py _bindings:="/path/to/your/yaml.yaml"
    ```

### Future Plans
The driver is extensible to any other input device, so that you can interface
a teach panel and jog your robot with ROS.

## Store Points

After moving the robot to the desired position, you can store the current position to file.

To save the current robot pose as ros_msg to a file use `rosrun pilz_teleoperation store_current_pose.py`.
The generated file can be included into your pilz_robot_programming script as follows:

```
#!/usr/bin/env python
from pilz_robot_programming import *
import points as pts
import rospy

__REQUIRED_API_VERSION__ = "1"    # API version
__ROBOT_VELOCITY__ = 0.5          # velocity of the robot

# main program
def start_program():
  
    # move to start point with joint values to avoid random trajectory
    r.move(Ptp(goal=pts.pick_pose, vel_scale=__ROBOT_VELOCITY__))

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    start_program()
```

while the generated points.py looks similar as:
```
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

pick_pose = Pose(
    position = Point(
      x = 0.07,
      y = 0.40,
      z = 0.50
    ),
    orientation = Quaternion(
      x = 1.0,
      y = 0.0,
      z = 0.0,
      w = 0.0
    )
)
```
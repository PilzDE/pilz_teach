# Teleoperation
Teleoperation (jogging) for a manipulator e.g. using a keyboard interface.

## Getting Started
Bring up your robot and jog-arm-server; see for example [prbt_jog_arm package](../prbt_jog_arm_support/README.md).

To start the keyboard frontend:

```
rosrun pilz_teleoperation key_teleop.py
```

To move the robot around, use keypad numbers as described in the on-screen instructions.

### Alternative Key Bindings

To use alternative keyboad bindings for the key_teleop driver do:
- create a separate config yaml file.
Use following Syntax:
    ```
    MovementBindings:
      'w': <Twist> (e.g. {"linear": {"x": 1.0}})
      '<CURSES_KEY_NAME>': <Twist> (e.g. KEY_LEFT)
    SettingBindings:
      '+': <SettingName> (e.g. USE_XZ_PLANE)
    Description: |-
      Displayed Text e.g. to show current bindings
      With the '|-' the text can contain multiple rows
    ```
    
    See [SetTeleopSettings.srv](./srv/SetTeleopSettings.srv) for available settings.
    
    See [Curses key names](https://www.gnu.org/software/guile-ncurses/manual/html_node/Getting-characters-from-the-keyboard.html)
    for name of special characters in curses.

- then start the node with the additional argument "_bindings"

    ```
    rosrun pilz_teleoperation key_teleop.py _bindings:="/path/to/your/yaml.yaml"
    ```

## Future Plans
The driver is extensible to any other input device, so that you can interface
a teach panel and jog your robot with ROS.

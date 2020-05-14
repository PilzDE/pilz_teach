# Teleoperation
Teleoperation (jogging) for a manipulator e.g. using a keyboard interface.

## Keyboard Teleoperation

### Getting Started
Bring up your robot and jog-arm-server; see for example [prbt_jog_arm package](../prbt_jog_arm_support/README.md).

To start the keyboard frontend:

```
roslaunch pilz_teleoperation key_teleop.launch
```

To move the robot around, use keyboard keys as described in the on-screen instructions.

### Teleoperation Settings
The teleoperation driver is parametrized by the 
[teleoperation_settings.yaml](../pilz_teleoperation/config/teleoperation_settings.yaml).
These Settings define the joints and target frames to toggle, as well as the velocities to which 
the driver will clamp the speed to. The `teleop_config_file` parameter can be modified to load a custom
settings file.

```
roslaunch pilt_teleoperation key_teleop.launch teleop_config_file:="/path/to/your/config.yaml"
```

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
    roslaunch pilz_teleoperation key_teleop.launch bindings_file:="/path/to/your/binding.yaml"
    ```

### Future Plans
The driver is extensible to any other input device, so that you can interface
a teach panel and jog your robot with ROS.

## Store Points

The position storage has moved to repository [pilz_industrial_motion](https://github.com/PilzDE/pilz_industrial_motion)

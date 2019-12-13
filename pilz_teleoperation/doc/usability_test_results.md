# General

- Mention in documentation, that the terminal in which teleop runs always needs to be focused
- Switching between rviz and terminal window is considered annoying
- Jointwise AND axiswise rotation requested for all bindings
- Units [m/s] and [rad/s] lie
- In description: rather use tables. And leave away quotation marks
- Rotation should in general be named rx ry rz

# key_teleop

## pros

- Switching between planes is used and considered a good idea
- Target frame concept is understood
- Well structured in general
- Mapping is good
- Moving diagonally in planes

## cons

- Joint jog velocity is not controlled by angular velocity, but by linear velocity
- Automatic plane-orthogonal orientation desired
- Jogging of "joints 2,3 and 5 simulatnious" should get an intuitively graspable name
- Axis rotation missing in description
- "Much text"
- User need much time to find certain controls in description
- Write "Next joint" instead of "Toggle to next joint" and so on
- Hard to understand without reading the description
- Concept of moving inside a plane seems to be confusing for some users

# key_teleop_single_dimensions

## pros

- Very intuitive
- Less text than key_teleop

## cons

- Key for toggling target frame is missing
- 'shift + char' for rotation missing in description

## suggestions

- Enable rotation througj typing the angles by hand
- Enable plane toggle
- Increase velocity with key-push-time and rather acceleration than velocity adjustable

# key_teleop_toggle_velocity

## pros

- Speed button is considered great

## cons
- Description of speed button crappy. Instead of uppercase letters for faster movement, we can simply write "shift: highspeed mode"
- Changing angular speed not implemented

## suggestions

- Add frame toggle
- Speed mode is always max velocity and only standard velocity is adjustable
- Use ctrl as speed button if possible
- Add plane toggling

# key_teleop_wasd

## pros

- Very intuitive for gamers

## cons

- Keys for rotation missing
- Very counterintuitive for non-gamers
- Toggling rotation axis considered not good
- Discrete velocities (contunious desired)

## suggestions

- Enable target frame toggling
- Logarithmic velocity toggles

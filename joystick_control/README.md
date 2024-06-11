# joystick_control package

This package allows steering the rover and the manipulator using one or more joystick controllers.

Steering can be switched between rover and manipulator using start/back buttons.

## Usage

### Launch files

- `single_joystick_steering.launch`

  ```bash
  $ roslaunch joystick_control single_joystick_steering.launch
  ```

  Launches a joystick steering module.

- `multi_joystick_steering.launch`

  ```bash
  $ roslaunch joystick_control multi_joystick_steering.launch
  ```

  Launches multiple joystick steering modules.

### Nodes

- `joy_node` (one or more with indexes)

  Parameters:

  - `~dev`
    - Type:       `string`
    - Dafault:    `/dev/input/js0`
  - `~deadzone`
    - Type:       `double`
    - Default:    `0.05`
    - Set:        `0.15`
  - `~coalesce_interval`
    - Type:       `double`
    - Default:    `0.001`
    - Set:        `0.02`
    - Unit:       `sec`
  - `~autorepeat_rate`
    - Type:       `double`
    - Default:    `0.0`
    - Set:        `50.0`
    - Unit:       `Hz`
  - `~default_trig_val`
    - Type:       `bool`
    - Default:    `false`
    - Set:        `true`

- `joystick_steering_node` (one or more with indexes)

  Parameters:

  - `~ID`
    - Type:       `int`
    - Default:    `0`

    Identification number of each joystick and corresponding node.

  - `~joy_message_topic_name`
    - Type:       `string`
    - Default:    `joy`

    Topic to subscribe messages from each node.

  - `~joystick_type`
    - Type:       `string`
    - Default:    `XBOX_XS`

    Type of used joystick controller included in `./config/joystick.yaml`.

  - `~child_mode`
    - Type:       `bool`
    - Default:    `false`

    Enables steering mode for non-skar people.

  - `~child_mode_inertia_i1`
    - Type:       `double`
    - Default:    `0.4444`

    The lower the inertia param, the longer it takes for the rover to accelerate. Affects rover's max acceleration.
  
  - `~child_mode_inertia_i2`
    - Type:       `double`
    - Default:    `0.0066`

    The lower the inertia param, the longer it takes for the rover to accelerate. Affects rover's acceleration speed.

### Config files

- `joystick.yaml`

  Sets the mapping of the joystick keys to the numbers in `joy` message.

- `steering_modes.yaml`

  Sets all linear and nonlinear scaling parameters for all steering modes of the rover and the manipulator.

## Topics

### Subscribed topics

- `/joy_0` [sensor_msgs/Joy]
- `/joy_1` [sensor_msgs/Joy]
- ... and so on (depending on the number of connected joysticks)

  Each instance gets position of individual axes and buttons of corresponding joystick.

### Published topics

- `/cmd_vel` [geometry_msgs/Twist]

  Sets linear and angular velocity of rover.

- `/cmd_manip` [geometry_msgs/Twist]

  Sets linear velocity in each direction and angular velocity in each axis of effector.

- `/cmd_grip` [std_msgs/Float64]

  Sets an effort of gripper clamp.

## Steering description

### Rover steering

In the normal mode:

- Right stick left/right - turn left/right
- Right stick up/down - go forward/back

In the tank mode:

- Left stick up/down - let left wheels go forward/back
- Right stick up/down - let right wheels go forward/back

In the gamer mode:

- Right trigger - go forward
- Left trigger - go back
- Right stick left/right - turn wheels (like in car kinematics)

This mode can be changed dynamically using the buttons on the right:

- Button 'A' - normal mode
- Button 'B' - tank mode
- Button 'X' - gamer mode

Configuration parameters of all these modes are included in `./config/steering_modes.yaml`.

In case of lost signal or mode change, the rover stops.

### Child mode
Child mode can be enabled globally (affects all rover steering modes). It adds inertia to rover's response to joystick steering, reducing acceleration. It ensures the rover is safe while controlled by inexperienced operators (children).

### Manipulator steering

In the inverse kinematics mode:

- Arrow pad up/down - move effector forward/back
- Arrow pad left/right - move effector left/right
- Right stick up/down - move effector up/down
- Left stick up/down - tilt effector
- Left stick left/right - rotate effector
- Left/right trigger - clamp claw

This mode can be changed dynamically using the buttons on the right:

- Button 'X' - inverse kinematics
- *... another mode can be easily added together with assigning a different button*

Configuration parameters of all these modes are included in `./config/steering_modes.yaml`.

In case of lost signal or mode change, the manipulator stops.

## Tests

To configure axes and buttons mapping for each joystick type, run:

```bash
$ ls /dev/input/
$ sudo jstest /dev/input/jsX
```

where 'X' is the displayed number of joystick.

Next compare the displayed data, reacting to the change of joystick keys states, with the numbers assigned to the keys in `./config/joystick.yaml`.

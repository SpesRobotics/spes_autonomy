# Move Behavior

> Based on [`mep3_navigation::MoveBehavior`](https://github.com/memristor/mep3/edit/main/mep3_navigation/src/move_behavior/README.md).

Primitive but flexible move behavior for visual navigation and industrial mobile robots.

## Frames

There are four important frames:
- We defined an objective as a `target` frame (2D pose) in the `global` frame.
- The robot (`base` frame) finds the target in the `odom` frame and continues motion in the `odom` frame.

Therefore, there are two stages:
- Once the command is received we calculate the `target` pose in the `odom` frame: $$ T_{odom}^{target} = (T_{global}^{odom})^{-1} * T_{global}^{target} $$
- In the control loop we regulate the position according to the error: $$ T_{base}^{target} = (T_{odom}^{base})^{-1} * T_{odom}^{target} $$

In the typical example `global` = `map` and `odom` = `odom`.
It means the robot moves to the absolute pose, but it uses the `odom` frame to avoid discrete pose jumps.

However, `global` + `odom` frames gives us a flexibility to achieve useful behaviors:
- If `global` = `base_link` and `odom` = `odom` then the robot moves relative to its current pose.
- If `global` = `marker_1` (a fiducial marker) and `odom` = `marker_1` then the robot moves to the marker pose (e.g. to pick the object). 
- However, if the robot loses the `marker_1` from its sight then we can set the following `global` = `marker_1` and `odom` = `odom`.

## Configuration

```yaml
move:
    ros__parameters:
        update_rate: 50
        command_timeout: 0.5

        # Can be overridden by the command.
        linear:
            kp: 0.5
            kd: 0.0
            max_velocity: 0.5
            max_acceleration: 0.5
            tolerance: 0.1
        angular:
            kp: 0.5
            kd: 0.0
            max_velocity: 0.5
            max_acceleration: 0.5
            tolerance: 0.1
```

## Examples

Move forward and stop:
```bash
ros2 topic pub -1 /move_command spes_msgs/msg/MoveCommand '{ "global_frame": "base_link", "odom_frame": "odom", "target": { "x": 0.2 }, "rotate_towards_goal": false, "rotate_at_goal": false }'
```

Keep moving forward:
```bash
ros2 topic pub -r1 /move_command spes_msgs/msg/MoveCommand '{ "global_frame": "base_link", "odom_frame": "odom", "target": { "x": 0.5 }, "rotate_towards_goal": false, "rotate_at_goal": false }'
```

Rotate in place:
```bash
ros2 topic pub -1 /move_command spes_msgs/msg/MoveCommand '{ "global_frame": "base_link", "odom_frame": "odom", "target": { "theta": 1.507 }, "rotate_towards_goal": false, "translate": false }'
```

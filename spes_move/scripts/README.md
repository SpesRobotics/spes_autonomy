## Using the ImageXYawRegulate Action

The ImageXYawRegulate action allowed for the regulation robot position relative to a image segment.

### Goal Parameters

1. `mode`:
   - Value 0 indicates "Y" mode for regulating the linear position.
   - Value 1 indicates "X" mode for regulating the angular position.

2. `tolerance`:
   - This parameter represents the tolerance for position regulation. Value between 0 and 1.

3. `overshoot_type`:
   - This parameter determines the type of overshoot behavior. It can take values "<=", ">=", "==", reflecting different overshoot modes.

4. `overshoot_value`:
   - This parameter represents the overshoot value used in the action. Its precise application depends on the value of `overshoot_type`. Allowed value between 0 and 1.

5. `image_segment`:
   - This parameter represents the image segment relative to which the robot's position will be regulated.



#### Example of sending a goal for position regulation in "X" (angular motion) mode with a tolerance of 0.05:

```
ros2 action send_goal /image_x_yaw_regulator/regulate spes_msgs/action/ImageXYawRegulate '{ "mode": 1, "tolerance": 0.05, "overshoot_type": "<=", "overshoot_value": 0.1, "image_segment": 0.72 }'
```


#### Example of sending a goal for position regulation in "Y" (linear motion) mode with a tolerance of 0.05:

```
ros2 action send_goal /image_x_yaw_regulator/regulate spes_msgs/action/ImageXYawRegulate '{ "mode": 0, "tolerance": 0.05, "overshoot_type": "<=", "overshoot_value": 0.1, "image_segment": 0.72 }'
```


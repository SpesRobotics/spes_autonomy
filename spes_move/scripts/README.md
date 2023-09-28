## ImageXYawRegulate

Regulates the robot's x & yaw according to object detections in an image.


#### Example of sending a goal for position regulation in "X" (angular motion) mode with a tolerance of 0.05:

```
ros2 action send_goal /image_x_yaw_regulator/regulate spes_msgs/action/ImageXYawRegulate '{ "mode": 1, "tolerance": 0.05, "type": "<", "overshoot_value": 0.1, "image_segment": 0.72 }'
```


#### Example of sending a goal for position regulation in "Y" (linear motion) mode with a tolerance of 0.05:

```
ros2 action send_goal /image_x_yaw_regulator/regulate spes_msgs/action/ImageXYawRegulate '{ "mode": 0, "tolerance": 0.05, "type": "<", "overshoot_value": 0.1, "image_segment": 0.72 }'
```


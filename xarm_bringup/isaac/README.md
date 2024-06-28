## Gripper Control
```bash
# Close
ros2 topic pub -1 /isaac/joint_command sensor_msgs/msg/JointState '{name: ["gripper_left_joint"], position: [-0.01]}'

# Open
ros2 topic pub -1 /isaac/joint_command sensor_msgs/msg/JointState '{name: ["gripper_left_joint"], position: [0.0]}'
```

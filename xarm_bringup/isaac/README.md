## Gripper Control
```bash
# Close
ros2 topic pub -1 /isaac/joint_command sensor_msgs/msg/JointState '{name: ["gripper_left_joint"], position: [-0.01]}'

# Open
ros2 topic pub -1 /isaac/joint_command sensor_msgs/msg/JointState '{name: ["gripper_left_joint"], position: [0.0]}'
```

## Servoing
Before using the cartesian API make sure the cartesian controller is running:
```bash
ros2 launch xarm_bringup lite6_cartesian_launch.py rviz:=false
```

Cartesian control example:
```bash
# Set setpoint
ros2 topic pub -r 10 /target_frame geometry_msgs/msg/PoseStamped '{header: {frame_id: "link_base"}, pose: {position: {x: 0.3, y: 0.0, z: 0.2}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}'

# Get current pose
ros2 topic echo /current_pose
```

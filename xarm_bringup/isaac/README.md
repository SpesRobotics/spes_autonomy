Before using the cartesian API make sure the cartesian controller is running:
```bash
ros2 launch xarm_bringup lite6_cartesian_launch.py rviz:=false
```

## Gripper Control
```bash
# Close
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray 'data: [-0.01]'

# Open
ros2 topic pub -1 /position_controller/commands std_msgs/msg/Float64MultiArray 'data: [0.0]'
```

## Servoing

Cartesian control example:
```bash
# Set setpoint
ros2 topic pub -r 10 /target_frame geometry_msgs/msg/PoseStamped '{header: {frame_id: "link_base"}, pose: {position: {x: 0.3, y: 0.0, z: 0.2}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}'

# Get current pose
ros2 topic echo /current_pose
```

## Moving to Joint Configuration
```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{activate_controllers: ["joint_trajectory_controller"], deactivate_controllers: ["cartesian_motion_controller"], strictness: 2, activate_asap: true}'

# You can use either action
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [
      { positions: [0, 0, 0, 0, 0, 0], time_from_start: { sec: 1, nanosec: 0 } },
    ]
  }
}"

# or topic
ros2 topic pub -1 /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6],
    points: [
        { positions: [0.00148, 0.06095, 1.164, -0.00033, 1.122, -0.00093], time_from_start: { sec: 1, nanosec: 0 } },
    ]
}"

ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController '{deactivate_controllers: ["joint_trajectory_controller"], activate_controllers: ["cartesian_motion_controller"], strictness: 2, activate_asap: true}'
```


## Gripper Control (Isaac Only)

These commands can be used only with Isaac when the cartesian controller is NOT running.

```bash
# Close
ros2 topic pub -1 /isaac/joint_command sensor_msgs/msg/JointState '{name: ["gripper_left_joint"], position: [-0.01]}'

# Open
ros2 topic pub -1 /isaac/joint_command sensor_msgs/msg/JointState '{name: ["gripper_left_joint"], position: [0.0]}'
```
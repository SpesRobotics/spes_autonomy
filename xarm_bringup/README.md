# xarm_bringup

## Overview

The `xarm_bringup` package provides tools and scripts for simulating and controlling the xArm robot in both simulated and real environments. Follow the instructions below to configure and run the appropriate scripts for each environment.

## Environment Setup

### Simulation Environment (SimEnv)

1. **Update the URDF Configuration**

   - Open the `xarm_bringup/urdf/lite6.urdf.xacro` file.
   - Locate the line defining `ros2_control_plugin` and change its value to `TopicBasedSystem`.

     ```xml
     <xacro:property name="ros2_control_plugin" value="TopicBasedSystem"/>
     ```

2. **Run the Scripts**

   - Generate episodes with the following command:

     ```sh
     ./xarm_bringup/scripts/episode_generator_picking
     ```

   - Record the generated episodes:

     ```sh
     ./xarm_bringup/scripts/episode_recorder
     ```

   - The recorded data will be saved in the `xarm_bringup/scripts/DATA` directory.

### Real Environment (RealEnv)

1. **Update the URDF Configuration**

   - Open the `xarm_bringup/urdf/lite6.urdf.xacro` file.
   - Locate the line defining `ros2_control_plugin` and change its value to `UFRobotSystemHardware`.

     ```xml
     <xacro:property name="ros2_control_plugin" value="UFRobotSystemHardware"/>
     ```

2. **Run the Scripts**

   - Manage episodes with the following command:

     ```sh
     ./xarm_bringup/scripts/episode_manager
     ```

   - Record the episodes:

     ```sh
     ./xarm_bringup/scripts/episode_recorder
     ```

   - The recorded data will be saved in the `xarm_bringup/scripts/DATA_REAL` directory.

## Notes

- Ensure that you have the necessary dependencies installed and that your ROS 2 environment is properly configured.
- For any issues or further assistance, please refer to the documentation or contact the maintainers.

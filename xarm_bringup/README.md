# xarm_bringup

## Overview

The `xarm_bringup` package provides tools and scripts for simulating and controlling the xArm robot in both simulated and real environments. Follow the instructions below to run data collection and saving data in `parquet` format.

## Environment Setup

### Simulation Environment (SimEnv)
1. **Run launch file**
    ```sh
      ros2 launch xarm_bringup lite6_cartesian_launch.py rviz:=false

    ```
2. **Run the Scripts**

   - Control simulated robot:

     ```sh
     ./xarm_bringup/scripts/episode_generator_picking
     ```

   - Record the generated episodes:

     ```sh
     ./xarm_bringup/scripts/episode_recorder
     ```

   - The recorded data will be saved in the `xarm_bringup/scripts/DATA` directory.

### Real Environment (RealEnv)

1. **Run launch file**
    ```sh
      ros2 launch xarm_bringup lite6_cartesian_launch.py rviz:=false sim:=false

    ```

2. **Run the Scripts**
   - Control the arm init pose, manage start and episode end:

     ```sh
     ./xarm_bringup/scripts/episode_manager
     ```

   - Record the episodes:

     ```sh
     ./xarm_bringup/scripts/episode_recorder
     ```

    - Run space mouse
     ```sh
     ./xarm_bringup/scripts/space_teleop
     ```

   - The recorded data will be saved in the `xarm_bringup/scripts/DATA_REAL` directory.


### Save data in parquet format
```sh
     ./xarm_bringup/scripts/save_parquet --filename DATA_REAL
```
File will be saved in path `DATA_REAL/parquest_output`

## Notes

- Ensure that you have the necessary dependencies installed


### Enable manual mode

- Run launch file
```sh
    ros2 launch xarm_bringup lite6_cartesian_launch.py rviz:=false sim:=false
```

- Run commands:

#Set mode need to call twice time

```sh
   ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 2}"
```

```sh
   ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 2}"
```

```sh
   ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```

**To return in normal mode run commands:**

```sh
   ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 "{data: 0}"
```

```sh
   ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 "{data: 0}"
```
- After this step need to run launch file again


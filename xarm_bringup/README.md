### SimEnv
Change ros2_control_plugin in `xarm_bringup/urdf/lite6.urdf.xacro` to `TopicBasedSystem` and than run:

```sh
./xarm_bringup/scripts/episode_generator_picking
```
```sh
./xarm_bringup/scripts/episode_recorder
```
Recorded data will be save in path `xarm_bringup/scripts/DATA`

### RealEnv

Change ros2_control_plugin in `xarm_bringup/urdf/lite6.urdf.xacro` to `UFRobotSystemHardware` and than run:

```sh
./xarm_bringup/scripts/episode_manager
```

```sh
./xarm_bringup/scripts/episode_recorder
```
Recorded data will be save in path `xarm_bringup/scripts/DATA_REAL`

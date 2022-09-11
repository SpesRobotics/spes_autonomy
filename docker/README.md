Build a container:
```bash
# Build the Docker image
make build

# Create a Docker container out of image
make run

# Start the Docker container
make exec
```

In container:
```bash
# Create a map
ros2 launch nav2_bringup slam_launch.py
rviz2

# Save the map
ros2 run nav2_map_server map_saver_cli -f /spesbot/assets/map

# Navigate
ros2 launch nav2_bringup bringup_launch.py map:=/spesbot/assets/map.yaml
ros2 launch nav2_bringup rviz_launch.py
```

Install dependencies:
```bash
sudo apt install git make curl
curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER

# Reboot PC to apply the docker group changes
sudo reboot 
```

Build a container:
```bash
# Build & run Docker (only the first time)
make build-pc run

# Attach the shell to the container
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

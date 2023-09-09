# Spes Autonomy

Spes Robotics autonomy packages, software infrastructure, and hardware platform.

## Installation

You can install as a typical ROS workspace, but we highly recommend using Docker: 

```bash
# Install Docker (and other dependencies) and reboot to apply the user group change
sudo apt install git make curl
curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER
sudo reboot 

# Clone the repo
git clone https://github.com/SpesRobotics/spes_autonomy.git $HOME/spes_autonomy

# Build and run the Docker image (only once)
make -f $HOME/spes_autonomy/docker/Makefile build-pc run

# Attach to a terminal
make -f $HOME/spes_autonomy/docker/Makefile exec
```

## Examples

Visual servoing with latency compensation:
```bash
ros2 launch spes_tests test_move_latency_compensation_launch.py
```

SLAM & Navigation:
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

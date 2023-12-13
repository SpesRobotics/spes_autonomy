#!/bin/bash

set -e

source /opt/ros/humble/setup.bash
source /${DOCKERUSER}/ros2_ws/install/local_setup.bash
source /${DOCKERUSER}/ros2_libs_ws/install/local_setup.bash

exec "$@"

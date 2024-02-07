# SpesBot Webots Simulation

Regenerate the Webots model:
```bash
pip3 install urdf2webots
xacro ${HOME}/ros2_ws/src/spesbot/spesbot_description/urdf/spesbot.xacro | python3 -m urdf2webots.importer --output=${HOME}/ros2_ws/src/spesbot/spesbot_webots/data/protos/SpesBot.proto --tool-slot=base_link
```

## Generate USD

First, generate URDF:
```bash
xacro lite6.urdf.xacro > lite6.urdf && \
    sed -i 's/package:\/\/xarm_bringup\/urdf\///g' lite6.urdf && \
    cp -r /spesbot/ros2_libs_ws/src/xarm_ros2/xarm_description/meshes/ xarm_description_meshes && \
    sed -i 's/package:\/\/xarm_description\/meshes\//xarm_description_meshes\//g' lite6.urdf
```

Then in Isaac URDF importer apply these configs:
- [x] Merge Fix Joints
- [x] Override Joint Dynamics
- [x] Convex Decomposition
- [x] Clear Stage

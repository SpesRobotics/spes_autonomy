# CUDA Accelerated Modules

In certain cases it is tricky to have hardware accelerated environment together with ROS modules.
In our experience, we found easier to separate them in different Docker environments.

## Object Tracker

Publishes detections from camera:
```bash
docker build -t spes-object-tracker-image .

docker run --tty --rm --net=host --privileged -v ${PWD}:/home spes-object-tracker-image --source 4
```

# Object Tracker

Publishes detections from camera:
```bash
docker build -t spes-object-tracker-image .

docker run --tty --rm --net=host --privileged -v ${PWD}:/home spes-object-tracker-image --source 4
```
UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
NVIDIA_GPU:=$(shell docker info | grep Runtimes | grep nvidia 1> /dev/null && echo '--runtime nvidia --gpus all' || echo '')
BASE:=spesbot
IMAGE:=spesbot-image

exec:
	docker exec -it spesbot-container bash
	
rebuild-pc:
	DOCKER_ARGS="--no-cache --pull" $(MAKE) build-pc
	
rebuild-sbc:
	DOCKER_ARGS="--no-cache --pull" $(MAKE) build-sbc

vnc:
	$(eval BASE=vnc)
	$(eval IMAGE=spesbot-vnc)
	@true

run:
	@docker container rm -f spesbot-container
	docker run \
		--net=host \
		--cap-add SYS_ADMIN \
		--name spesbot-container \
		--privileged \
		--restart unless-stopped \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/spesbot/.host/.Xauthority:ro \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v ${ROOT_DIR}/assets:/spesbot/assets \
		-v ${ROOT_DIR}/../../..:/spesbot/ros2_ws \
		-v /dev/bus/usb:/dev/bus/usb \
		-v /dev:/dev:rw \
		-v /dev/dri:/dev/dri:ro \
		-d -it ${IMAGE}

build-pc:
	docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.pc -t ${IMAGE} ${DOCKER_ARGS} --build-arg UID=${UID}

build-sbc:
	docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.sbc -t ${IMAGE} ${DOCKER_ARGS} --build-arg UID=${UID}

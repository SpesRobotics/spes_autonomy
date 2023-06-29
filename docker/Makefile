UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
NVIDIA_GPU:=$(shell docker info | grep Runtimes | grep nvidia 1> /dev/null && echo '--runtime nvidia --gpus all' || echo '')
DOCKERUSER:=spesbot
BASENAME:=spesbot
DOCKERUSERCOMMENT:="SpesBot (Spes Robotics)"

exec:
	docker exec -it ${BASENAME}-container bash
	
rebuild-pc:
	DOCKER_ARGS="--no-cache --pull" $(MAKE) build-pc
	
rebuild-sbc:
	DOCKER_ARGS="--no-cache --pull" $(MAKE) build-sbc

run:
	@docker container rm -f ${BASENAME}-container
	docker run \
		--net=host \
		--ipc=host \
		--cap-add SYS_ADMIN \
		--name ${BASENAME}-container \
		--privileged \
		--restart unless-stopped \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/${DOCKERUSER}/.Xauthority:ro \
		-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
		-v /dev/dri:/dev/dri:ro \
		-v ${ROOT_DIR}/..:/${DOCKERUSER}/ros2_ws/src/${BASENAME} \
		-v ${ROOT_DIR}/assets:/${DOCKERUSER}/assets \
		-v /dev:/dev:rw \
		-d -it ${BASENAME}-image

build-pc:
	docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.pc -t ${BASENAME}-image ${DOCKER_ARGS} --build-arg UID=${UID} --build-arg DOCKERUSER=${DOCKERUSER} --build-arg DOCKERUSERCOMMENT=${DOCKERUSERCOMMENT}

build-sbc:
	docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.sbc -t ${BASENAME}-image ${DOCKER_ARGS} --build-arg UID=${UID} --build-arg DOCKERUSER=${DOCKERUSER} --build-arg DOCKERUSERCOMMENT=${DOCKERUSERCOMMENT}

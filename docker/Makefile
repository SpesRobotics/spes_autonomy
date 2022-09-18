UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

exec:
	docker exec -it spesbot-container bash

run:
	@docker container rm -f spesbot-container
	docker run \
		--net=host \
		--cap-add SYS_ADMIN \
		--name spesbot-container \
		--privileged \
		--restart unless-stopped \
		--runtime nvidia \
		-e DISPLAY=${DISPLAY} \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v ${ROOT_DIR}/assets:/spesbot/assets \
		-v ${ROOT_DIR}/../../..:/spesbot/ros2_ws \
		-d -it spesbot-image

build-pc:
	docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.pc -t spesbot-image --build-arg UID=${UID}

build-sbc:
	docker build ${ROOT_DIR} -f ${ROOT_DIR}/Dockerfile.sbc -t spesbot-image --build-arg UID=${UID}

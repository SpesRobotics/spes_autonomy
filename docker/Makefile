UID:=$(shell id -u)

exec:
	docker exec -it spesbot-container bash

run:
	@docker container rm -f spesbot-container
	docker run \
		--net=host \
		--cap-add SYS_ADMIN \
		--name spesbot-container \
		--privileged \
		--rm \
		--runtime nvidia \
		-e DISPLAY=${DISPLAY} \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v ${PWD}/assets:/spesbot/assets \
		-v ${PWD}/../../..:/spesbot/ros2_ws \
		-d -it spesbot-image

build-pc:
	docker build . -f Dockerfile.pc -t spesbot-image --build-arg UID=${UID}

build-sbc:
	docker build . -f Dockerfile.sbc -t spesbot-image --build-arg UID=${UID}

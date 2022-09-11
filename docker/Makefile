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
		-d -it spesbot-image

build:
	docker build . -t spesbot-image --build-arg UID=${UID}

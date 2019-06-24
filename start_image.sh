#!/bin/bash

nvidia-docker \
	run \
	-it \
	--rm \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v "$PWD:/SwarmiesRL" \
	--privileged \
	--device /dev/dri \
	-e DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	swarmiesrl:latest


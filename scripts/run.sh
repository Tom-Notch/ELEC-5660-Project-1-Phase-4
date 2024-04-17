#!/usr/bin/env bash
#
# Created on Wed Oct 25 2023 11:03:46
# Author: Mukai (Tom Notch) Yu
# Email: yumukai@huawei.com, topnotchymk@gmail.com
# Affiliation: HUAWEI Technologies Co. Ltd., Hong Kong Research Center, Design Automation Lab
#
# Copyright Ⓒ 2023 Mukai (Tom Notch) Yu
#

. "$(dirname "$0")"/variables.sh

xhost +local:*

if [ ! -f "$XAUTH" ]; then
	touch "$XAUTH"
	xauth_list=$(xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/')
	if [ -n "$xauth_list" ]; then
		echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
	fi
	chmod a+r "$XAUTH"
fi

if [ "$(docker ps -a -q -f name="$CONTAINER_NAME")" ]; then
	echo "A container with name ""$CONTAINER_NAME"" is running, force removing it"
	docker rm -f "$CONTAINER_NAME"
	echo "Done"
fi

docker run \
	--name "$CONTAINER_NAME" \
	--hostname "$(hostname)" \
	--privileged \
	--cpus "$AVAILABLE_CORES" \
	--network host \
	--ipc host \
	--pid host \
	--ulimit core=-1 \
	--group-add audio \
	--group-add video \
	--volume=":" \
	-e DISPLAY="$DISPLAY" \
	-e XAUTHORITY="$XAUTH" \
	-e QT_X11_NO_MITSHM=1 \
	-v /var/lib/systemd/coredump/:/cores \
	-v "$XSOCK":"$XSOCK" \
	-v "$XAUTH":"$XAUTH" \
	-v "$HOME"/.Xauthority:"$CONTAINER_HOME_FOLDER"/.Xauthority:rw \
	-v "$(dirname "$0")"/..:"$CONTAINER_HOME_FOLDER"/"$CODE_FOLDER" \
	-w "$CONTAINER_HOME_FOLDER"/"$CODE_FOLDER" \
	--rm \
	-itd "$DOCKER_USER"/"$IMAGE_NAME":"$IMAGE_TAG"

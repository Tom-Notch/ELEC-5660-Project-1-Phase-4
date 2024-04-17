#!/usr/bin/env bash
#
# Created on Wed Oct 25 2023 11:04:47
# Author: Mukai (Tom Notch) Yu
# Email: yumukai@huawei.com, topnotchymk@gmail.com
# Affiliation: HUAWEI Technologies Co. Ltd., Hong Kong Research Center, Design Automation Lab
#
# Copyright Ⓒ 2023 Mukai (Tom Notch) Yu
#

. "$(dirname "$0")"/variables.sh

docker buildx build \
	--platform=linux/amd64,linux/arm64 \
	--build-context home-folder-config="$(dirname "$0")"/../docker/build-context/home-folder-config \
	-t "$DOCKER_USER"/"$IMAGE_NAME":"$IMAGE_TAG" \
	- <"$(dirname "$0")"/../docker/"$IMAGE_TAG"/Dockerfile

#!/usr/bin/env bash
#
# Created on Wed Oct 25 2023 11:04:27
# Author: Mukai (Tom Notch) Yu
# Email: yumukai@huawei.com, topnotchymk@gmail.com
# Affiliation: HUAWEI Technologies Co. Ltd., Hong Kong Research Center, Design Automation Lab
#
# Copyright Ⓒ 2023 Mukai (Tom Notch) Yu
#

. "$(dirname "$0")"/variables.sh

docker exec --privileged -it "$CONTAINER_NAME" pkill -f zsh

docker rm -f "$CONTAINER_NAME"

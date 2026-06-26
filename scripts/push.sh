#!/usr/bin/env bash
#
# Created on Wed Oct 25 2023 11:03:54
# Author: Mukai (Tom Notch) Yu
# Email: yumukai@huawei.com, tomnotch.yu@gmail.com
# Affiliation: HUAWEI Technologies Co. Ltd., Hong Kong Research Center, Design Automation Lab
#
# Copyright Ⓒ 2023 Mukai (Tom Notch) Yu
#

. "$(dirname "$0")"/variables.sh

docker push "$DOCKER_USER"/"$IMAGE_NAME":"$IMAGE_TAG"

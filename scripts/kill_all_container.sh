#!/usr/bin/env bash
#
# Created on Wed Oct 25 2023 11:04:37
# Author: Mukai (Tom Notch) Yu
# Email: yumukai@huawei.com, topnotchymk@gmail.com
# Affiliation: HUAWEI Technologies Co. Ltd., Hong Kong Research Center, Design Automation Lab
#
# Copyright Ⓒ 2023 Mukai (Tom Notch) Yu
#

. "$(dirname "$0")"/variables.sh

echo "Removing all containers"

for id in $(docker ps -aq); do
	echo "Removing container $id"
	docker rm -f "$id"
done

echo "Done"

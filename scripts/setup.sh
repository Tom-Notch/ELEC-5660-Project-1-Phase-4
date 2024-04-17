#!/usr/bin/env bash
#
# Created on Wed Oct 25 2023 11:03:38
# Author: Mukai (Tom Notch) Yu
# Email: yumukai@huawei.com, topnotchymk@gmail.com
# Affiliation: HUAWEI Technologies Co. Ltd., Hong Kong Research Center, Design Automation Lab
#
# Copyright Ⓒ 2023 Mukai (Tom Notch) Yu
#

. "$(dirname "$0")"/variables.sh

sudo apt update

echo "Installing clang-tidy"
sudo apt install -y clang-tidy

echo "Installing pre-commit"
pip3 install pre-commit

echo "adding pre-commit executable to path"
echo "export PATH=~/.local/bin:$PATH" | sudo tee -a "${HOME}"/.bashrc
. "${HOME}"/.bashrc

echo "Setting up pre-commit"
pre-commit install

echo "Done!"

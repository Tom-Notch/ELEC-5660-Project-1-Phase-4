#!/bin/sh

. "$(dirname "$0")"/variables.sh

. "$(dirname "$0")"/run.sh

docker exec -it "${CODE_FOLDER}" /bin/zsh -c "source ~/.zshrc && \
                                              export ROS_MASTER_URI=http://192.168.1.100:11311/ && \
                                              export ROS_IP=192.168.1.100 && \
                                              roslaunch core_automation core_automation_node.launch"

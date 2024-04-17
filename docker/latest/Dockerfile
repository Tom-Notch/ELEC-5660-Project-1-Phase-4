# Do not add --platform=linux/blabla since this is intended for multiplatform builds
FROM tomnotch/bipvrobotics-base-image:ros1
ENV HOME_FOLDER=/root
WORKDIR $HOME_FOLDER/

# Fix apt install stuck problem
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# update all obsolete packages to latest, install sudo, and cleanup
RUN apt update -o Acquire::Check-Valid-Until=false -o Acquire::AllowInsecureRepositories=true -o Acquire::AllowDowngradeToInsecureRepositories=true && \
    apt full-upgrade -y && \
    apt install -y sudo ca-certificates && \
    apt autoremove -y && \
    apt autoclean -y

RUN apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs ros-noetic-vrpn ros-noetic-realsense2-camera

RUN rm -rf /var/lib/apt/lists/*

ENV CODE_FOLDER=elec-5660-project-1-phase-4
RUN echo "source ${HOME_FOLDER}/${CODE_FOLDER}/devel/setup.zsh" >> ${HOME_FOLDER}/.zshrc && \
    echo "source ${HOME_FOLDER}/${CODE_FOLDER}/devel/setup.bash" >> ${HOME_FOLDER}/.bashrc

# Set the default shell to zsh
SHELL [ "/bin/zsh", "-c" ]

# # move fixuid config
# RUN mv $HOME_FOLDER/fixuid-config.yml /etc/fixuid/config.yml

# Entrypoint command
# ENTRYPOINT [ "/bin/sh" , "-c", "fixuid; /bin/zsh" ]
ENTRYPOINT [ "/bin/zsh" ]

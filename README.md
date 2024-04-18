# ELEC 5660 Project 1 Phase 4

[![Continuous Integration](https://github.com/Tom-Notch/ELEC-5660-Project-1-Phase-4/actions/workflows/ci.yml/badge.svg)](https://github.com/Tom-Notch/ELEC-5660-Project-1-Phase-4/actions/workflows/ci.yml) [![pre-commit](https://github.com/Tom-Notch/ELEC-5660-Project-1-Phase-4/actions/workflows/pre-commit.yml/badge.svg)](https://github.com/Tom-Notch/ELEC-5660-Project-1-Phase-4/actions/workflows/pre-commit.yml)

## Pre-requisites

- [Docker](https://docs.docker.com/get-docker/)
- [Git LFS](https://git-lfs.github.com/) (for downloading the bag file)

## Setup

1. Clone the repository

   ```bash
   git clone --recursive https://github.com/Tom-Notch/ELEC-5660-Project-1-Phase-4.git
   ```

1. Run pre-built docker image

   ```bash
   ./scripts/run.sh
   ```

   or

   ```bash
   docker compose up -d
   ```

1. Attach to the running container

   ```bash
   docker attach elec-5660-project-1-phase-4
   ```

1. Build & Source

   ```bash
   catkin build
   source devel/setup.bash
   ```

1. Run

   ```bash
   roslaunch [package_name] [launch_file_name]
   ```

## Directory Structure

Videos are under [video/](video/) folder

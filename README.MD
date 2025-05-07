# Bender Nav2

## Table of contents
- [Bender Nav2](#bender-nav2)
  - [Table of contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Requirements](#requirements)
  - [How to run](#how-to-run)

## Introduction

This proyect implements NAV2 stack in Bender, using docker containers.

## Requirements

Before running this proyect, you must install:

1. [Docker](https://docs.docker.com/engine/install/)

The rest of requirements are installed automatically when the docker image is built.

## How to run

```bash
cd ~
git clone https://github.com/uchile-robotics/bender_bringup.git
cd bender_bringup
git checkout feat-jazzy
cd ..
mkdir bender_jazzy && cd bender_jazzy
git clone https://github.com/uchile-robotics/bender_nav2.git
cd ~/bender_nav2/dockerfile/Rosaria
sudo docker compose build rosaria2_container
```

Then you must wait for the docker image to build. After that step, check if Bender's base is connected to your PC via USB (and the hokuyo)

```bash
cd dockerfile/rosaria2
sudo docker compose up -d --remove-orphans
sudo docker exec -it rosaria2_container /bin/bash
```

In the same terminal that you ran the last command, after connecting bender's base and lidar you must run:

```bash
colcon build --symlink-install
ros2 launch bender_bringup bender_basic.launch.py

```




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
git clone https://github.com/uchile-robotics/bender_nav2.git
cd bender_nav2/dockerfile/Rosaria
sudo docker build -t rosaria2 .
```
Then you must wait for the docker image to build. After that step, check if Bender's base is connected to your PC via USB

```bash
docker run -dit --device=/dev/ttyUSB0 --network host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY --name rosaria2_container rosaria2

sudo docker exec -it rosaria2_container /bin/bash
```

In the same terminal that you ran the last command, you must run:

```bash
ros2 launch rosaria2 rosaria2.launch.py # inside the same terminal that you ran the last command 

```
Then, in another terminal, run:
```bash
sudo docker exec -it rosaria2_container /bin/bash
```
Inside that terminal run `ros2 topic list` and search for a topic called something along the lines of  `/AriaCoda/cmd_vel`. After that, you must change the name of the topic that teleop_twist_keyboard is publishing to. In order to do that, you must run this command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=<my_cmd_vel>

```

With that you can control Bender's base


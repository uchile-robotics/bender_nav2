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
mkdir bender_jazzy && cd bender_jazzy
git clone https://github.com/uchile-robotics/bender_nav2.git
cd bender_nav2/dockerfile/Rosaria
sudo docker compose build rosaria2_container
```
Then you must set up the communication with the docker container
```bash
mkdir -p ~/.config/ros
cd ~/bender_jazzy/bender_nav2
cp cyclonedds.xml ~/.config/ros
export CYCLONEDDS_URI=${HOME}/.config/ros/cyclonedds.xml
```
Then you must wait for the docker image to build. After that step, check if Bender's base is connected to your PC via USB

```bash
cd dockerfile/rosaria2
sudo docker compose up -d --remove-orphans
sudo docker exec -it rosaria2_container /bin/bash
```

In the same terminal that you ran the last command, you must run:

```bash
ros2 launch rosaria2 rosaria2.launch.py

```
Connect the hokuyo lidar to your pc, then run this command
```bash
sudo docker exec -it rosaria2_container /bin/bash
ros2 launch urg_node2 urg_node2.launch.py

```

Sahian, queda pendiente definir la pose del hokuyo en la base (la transformada, si le preguntas al gonz va a cachar)
ĺuego de eso puedes partir con el gmapping, para eso para estar más comodo puedes usar el paquete teleop_twist_joy
lo tendrías que instalar dentro del contenedor o por fuera, da lo mismo.

esopo

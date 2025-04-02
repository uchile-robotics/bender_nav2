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
sudo docker build -t ariacoda .
```
Then you must wait for the docker image to build. After that step, check if the pioneer is connected to your PC.

```bash
sudo docker run -dit --device=/dev/ttyUSB0 --name ariacoda_container ariacoda tail -f /dev/null

sudo docker start -i ariacoda_container

sudo docker exec -it ariacoda_container /bin/bash
```

In the same terminal that you ran the last command, you must run:

```bash
roscore
```
Then, in another terminal, run:
```bash
sudo docker exec -it ariacoda_container /bin/bash
rosrun rosaria RosAria _port:=/dev/ttyUSB0 # inside the same terminal that you ran the last command 
```

With that you can control Bender's base

TODO: queda pendiente ejecutar 2 terminales, 1 con roscore y otra con 
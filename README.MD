# Bender Nav2

## Table of contents
- [Bender Nav2](#bender-nav2)
  - [Table of contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Requirements](#requirements)
  - [Instalation](#instalation)
    - [Steps to create the `.netrc` file](#steps-to-create-the-netrc-file)
    - [Image building](#image-building)
  - [How to run](#how-to-run)
  
## Introduction

This proyect implements NAV2 stack in Bender, using docker containers.

## Requirements

Before running this proyect, you must install:

1. [Docker](https://docs.docker.com/engine/install/)

The rest of requirements are installed automatically when the docker image is built.

## Instalation


```bash
cd ~
git clone https://github.com/uchile-robotics/bender_bringup.git
cd bender_bringup
git checkout feat-jazzy
```
To prevent rate limit errors (HTTP 429) when running the `rosdep` command inside the Docker container, you need to authenticate requests to GitHub by creating a `.netrc` file.

### Steps to create the `.netrc` file

1. Generate a GitHub Personal Access Token (PAT) with minimal public_repo permissions. You can follow this official guide to create the token: [Creating a personal access token](https://docs.github.com/es/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-personal-access-token-classic).
2. Create the `.netrc` file with the following content, replacing `<your_github_token>` with your generated token.


```bash
printf "machine raw.githubusercontent.com\nlogin GITHUB_TOKEN\npassword <your_github_token>" > ~/.docker-github-netrc
sudo chmod 400 ~/.docker-github-netrc
```

### Image building
Now you will build the docker image

```bash
mkdir bender_jazzy && cd bender_jazzy
git clone https://github.com/uchile-robotics/bender_nav2.git
cd ~/bender_nav2/dockerfile/Rosaria
sudo docker compose build rosaria2_container
```
## How to run

Then you must wait for the docker image to build. After that step, check if Bender's base is connected to your PC via USB (and the hokuyo)

```bash
cd dockerfile/rosaria2
sudo docker compose up -d --remove-orphans
sudo docker exec -it rosaria2_container /bin/bash
```

In the same terminal that you ran the last command, after connecting bender's base and lidar you must run:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch bender_bringup bender_basic.launch.py

```




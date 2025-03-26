ARG BASE_OS=ubuntu:20.04
FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive
ARG NTRYPT=service ssh restart && bash

# Update system packages
RUN apt-get update && apt-get -y full-upgrade

# Install base dependencies
RUN apt-get update && apt-get install -y \
    git wget dbus-x11 lxqt-core tigervnc-standalone-server openssh-server \
    vim curl python3-pip software-properties-common tmux make g++ doxygen build-essential

# ROS Noetic installation
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-ros-base

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-empy \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-joy-listener \
    libfftw3-dev \
    python3-pip

# Initialize rosdep and update
RUN rosdep init && rosdep update
RUN pip install mock termcolor
# Clone and build ROS workspace with custom packages
WORKDIR /home
RUN mkdir -p bender_core/src && \
    cd bender_core/src &&\
    git clone https://github.com/amor-ros-pkg/rosaria.git && \
    git clone https://github.com/reedhedges/AriaCoda.git && \
    git clone https://github.com/uchile-robotics/bender_core.git

RUN sed -i '/<exec_depend>joystick_drivers<\/exec_depend>/d' /home/bender_core/src/bender_core/bender_joy/package.xml


# Install AriaCoda
RUN cd /home/bender_core/src/AriaCoda && make clean && make -j2

# Build the ROS workspace
WORKDIR /home/bender_core
RUN rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y
WORKDIR /home
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /home/bender_core && catkin_make"

##########################################################
# SSH setup

RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config
RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN echo "root:docker" | chpasswd
RUN usermod -a -G dialout root
EXPOSE 22
ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]


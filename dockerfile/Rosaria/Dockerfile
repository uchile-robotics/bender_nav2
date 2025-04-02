ARG BASE_OS=ubuntu:20.04
FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive
ARG NTRYPT=service ssh restart && bash

# Update system packages
RUN apt-get update && apt-get -y full-upgrade

# Install base dependencies
RUN apt-get update && apt-get install -y \
    git wget dbus-x11 lxqt-core tigervnc-standalone-server openssh-server \
    vim curl nano snap python3-pip software-properties-common tmux make g++ doxygen build-essential

# ROS Noetic installation
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-ros-base

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-empy \
    ros-noetic-map-server ros-noetic-amcl ros-noetic-urg-node ros-noetic-gmapping ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-tf2 ros-noetic-controller-manager ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
    ros-noetic-teleop-twist-keyboard 

# Initialize rosdep and update
RUN rosdep init && rosdep update

# Clone and build ROS workspace with custom packages
WORKDIR /home
RUN mkdir -p rosaria/src && \
    cd rosaria/src &&\
    git clone https://github.com/amor-ros-pkg/rosaria.git && \
    git clone https://github.com/reedhedges/AriaCoda.git 
    # git clone https://github.com/pengtang/rosaria_client.git

# Install AriaCoda
RUN cd /home/rosaria/src/AriaCoda && make clean && make -j2

# Build the ROS workspace
WORKDIR /home/rosaria
RUN rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"


RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config
RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN echo "root:docker" | chpasswd
EXPOSE 22

ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]
RUN echo "source /opt/ros/noetic/setup.bash\nsource /home/rosaria/devel/setup.bash" >> /root/.bashrc

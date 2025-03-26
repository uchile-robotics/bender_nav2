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

# CUDA environment setup
# ENV PATH=/usr/local/cuda/bin:$PATH
# ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# ROS Noetic installation
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-ros-base

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-empy \
    ros-noetic-map-server ros-noetic-amcl ros-noetic-urg-node ros-noetic-gmapping ros-noetic-urdf ros-noetic-xacro ros-noetic-tf2 \
    ros-noetic-controller-manager ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher

# Initialize rosdep and update
RUN rosdep init && rosdep update

# Clone and build ROS workspace with custom packages
WORKDIR /home
# RUN git clone https://github.com/Uchile-Lab-Vision-Computacional/pioneer_ws.git
RUN mkdir -p rosaria/src && \
    cd rosaria/src &&\
    git clone https://github.com/amor-ros-pkg/rosaria.git && \
    git clone https://github.com/reedhedges/AriaCoda.git

# Install AriaCoda
RUN cd /home/rosaria/src/AriaCoda && make clean && make -j2

# Build the ROS workspace
WORKDIR /home/rosaria
RUN rosdep install --from-paths src --ignore-src --rosdistro=noetic -r -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

##########################################################
# Install Python 3.10 and create a virtual environment
# RUN add-apt-repository ppa:deadsnakes/ppa && apt-get update
# RUN apt-get install -y python3.10 python3.10-venv python3.10-dev
# RUN python3.10 -m venv /home/venv

# Activate the virtual environment and upgrade pip inside the environment
# RUN /bin/bash -c "source /home/venv/bin/activate && python -m pip install --upgrade pip"

# Set the PATH to use the Python 3.10 virtual environment by default
# ENV VIRTUAL_ENV=/home/venv
# ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install PyTorch (using the virtual environment)
# RUN /bin/bash -c "source /home/venv/bin/activate && pip install torch torchvision torchaudio defusedxml"

# Install custom ROS packages (inside Python 3.10 virtual environment)
# WORKDIR /home
# COPY offline2online-rl ./offline2online-rl
# RUN /bin/bash -c "source /home/venv/bin/activate && cd offline2online-rl && pip install -e ."
# NOTA: deberia instalar pyyaml y rospkg pero esta en las dependencias del repo offline2online

##########################################################
# SSH setup

RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config
RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN echo "root:docker" | chpasswd
EXPOSE 22

# COPY ./ros_entrypoint.sh ./ros_entrypoint.sh
# SSH and bash entrypoint
ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]

# NVIDIA container runtime and libraries
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
# ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra

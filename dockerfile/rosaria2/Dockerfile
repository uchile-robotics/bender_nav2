ARG BASE_OS=nvcr.io/nvidia/cuda:12.5.1-base-ubuntu24.04

FROM ${BASE_OS}

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y full-upgrade

# locale

RUN apt-get update && apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# base dependencies
RUN apt update && apt upgrade -y
RUN apt-get install -y git wget dbus-x11 lxqt-core tigervnc-standalone-server openssh-server vim curl python3-pip

# ROS2


RUN apt-get install -y software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get -y full-upgrade

RUN apt-get install -y ros-jazzy-desktop

#ROS2 dev tools
RUN apt-get update && apt-get install -y \
    python3-flake8-blind-except \
    python3-flake8-class-newline \
    python3-flake8-deprecated \
    python3-mypy \
    python3-pip \
    python3-pytest \
    python3-pytest-cov \
    python3-pytest-mock \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    python3-pytest-runner \
    python3-pytest-timeout \
    ros-dev-tools \
    doxygen \
    ros-jazzy-teleop-twist-keyboard
    


WORKDIR /home
RUN mkdir -p base_controller/src && cd base_controller/src
RUN git clone https://github.com/reedhedges/AriaCoda.git
WORKDIR /home/AriaCoda
RUN make -j 
RUN make install


# RUN export ARIA_DIR=/usr/local/Aria
# RUN export CMAKE_PREFIX_PATH=$ARIA_DIR:$CMAKE_PREFIX_PATH
    
WORKDIR /home/base_controller/src
RUN git clone https://github.com/ruipaulorocha/rosaria2.git 
WORKDIR /home/base_controller
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-clean-cache --symlink-install"
RUN echo "source /opt/ros/jazzy/setup.bash\nsource /home/base_controller/install/setup.bash" >> /root/.bashrc


# Gazebo

# RUN apt-get update && apt-get -y full-upgrade
# RUN apt-get install -y ros-jazzy-ros-gz

# PyTorch
# RUN pip3 install torch torchvision torchaudio --break-system-packages

# MuJoCo
# RUN pip3 install mujoco --break-system-packages

# authorize SSH connection with root account
RUN sed -i 's/#PermitRootLogin .*/PermitRootLogin yes/' /etc/ssh/sshd_config

RUN echo "root:docker"|chpasswd

EXPOSE 22

ENTRYPOINT ["bash", "-c", "service ssh restart && exec bash"]

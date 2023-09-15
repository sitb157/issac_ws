FROM nvidia/opengl:base-ubuntu20.04

RUN apt-get -y update && apt-get install -y \
    iputils-ping \
    net-tools \
    wget \
    git
    
# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=galactic
ENV AMENT_PREFIX_PATH=/opt/ros/galactic
ENV COLCON_PREFIX_PATH=/opt/ros/galactic
ENV LD_LIBRARY_PATH=/opt/ros/galactic/lib
ENV PATH=/opt/ros/galactic/bin:$PATH
ENV PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    wget \ 
    lsb-release \
    sudo \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y \
    ros-galactic-desktop \
    python3-argcomplete \
    python3-pip \
    python3-rosdep \
    python3-setuptools \ 
    python3-vcstool \
    python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

# Install Cartographer
WORKDIR /root
RUN apt-get update && apt-get install -y && rm -rf /var/lib/apt/lists/*
COPY cartographer cartographer
RUN cartographer/scripts/install_debs_cmake.sh && rm -rf /var/lib/apt/lists/*
RUN cartographer/scripts/install_abseil.sh && rm -rf /var/lib/apt/lists/*
RUN cartographer/scripts/install_cartographer_cmake.sh

# Install Dependency packages
RUN apt-get update
RUN apt-get install -y \
    ros-galactic-joint-state-publisher \
    ros-galactic-cartographer-ros \
    ros-galactic-nav2* \
    ros-galactic-control-msgs \
    ros-galactic-moveit*

RUN mkdir -p /root/ros/workspace/ros2_workspace/src
COPY workspace/ros2_workspace/src /root/ros/workspace/ros2_workspace/src

# Install and Set up Vim
RUN apt-get install -y vim
WORKDIR /root/
RUN git clone https://github.com/sitb157/dotfiles.git
WORKDIR /root/dotfiles
RUN bash -c "./setup_dotfiles.sh"

RUN echo "source /opt/ros/galactic/setup.sh" >> /root/.bashrc
WORKDIR /root/ros/workspace/ros2_workspace

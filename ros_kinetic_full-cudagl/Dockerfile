# CODE ATTRIBUTION NOTICE =====================================================
# This Dockerfile is based off of the Dockerfile for kinetic-ros-core available
# at:
# https://github.com/osrf/docker_images/blob/49d22242e02e9c541a9e85b657e1785617b6f470/ros/kinetic/ubuntu/xenial/ros-core/Dockerfile
# END CODE ATTRIBUTION NOTICE =================================================
FROM nvidia/cudagl:9.0-base-ubuntu16.04

# install packages
RUN apt-get update && apt-get install -q -y \
  build-essential \
  cuda-command-line-tools-9-0 \
  cuda-cublas-9-0 \
  cuda-cufft-9-0 \
  cuda-curand-9-0 \
  cuda-cusolver-9-0 \
  cuda-cusparse-9-0 \
  curl \
  libcudnn7=7.2.1.38-1+cuda9.0 \
  libnccl2=2.2.13-1+cuda9.0 \
  libfreetype6-dev \
  libhdf5-serial-dev \
  libpng12-dev \
  libzmq3-dev \
  pkg-config \
  python3 \
  python3-dev \
  rsync \
  software-properties-common \
  unzip \
  dirmngr \
  gnupg2 \
  lsb-release \
  libusb-dev \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
  apt-get install nvinfer-runtime-trt-repo-ubuntu1604-4.0.1-ga-cuda9.0 && \
  apt-get update && \
  apt-get install libnvinfer4=4.1.2-1+cuda9.0

RUN curl -O https://bootstrap.pypa.io/get-pip.py && \
    python3 get-pip.py && \
    rm get-pip.py

RUN ln -s -f /usr/bin/python3 /usr/bin/python

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    ros-kinetic-robot-localization \
    ros-kinetic-usb-cam \
    ros-kinetic-rosbridge-server \
    ros-kinetic-hector-gazebo-plugins \
    ros-kinetic-joystick-drivers \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
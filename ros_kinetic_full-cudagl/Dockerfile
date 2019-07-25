# CODE ATTRIBUTION NOTICE =====================================================
# This Dockerfile is based off of the Dockerfile for kinetic-ros-core available
# at:
# https://github.com/osrf/docker_images/blob/49d22242e02e9c541a9e85b657e1785617b6f470/ros/kinetic/ubuntu/xenial/ros-core/Dockerfile
# END CODE ATTRIBUTION NOTICE =================================================
FROM tensorflow-cudagl:1.12.3

RUN apt-get update && apt-get install -q -y \
  dirmngr \
  gnupg2 \
  lsb-release \
  && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

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
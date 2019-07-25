FROM ros_kinetic_full-cudagl:latest

RUN apt-get update && apt-get install -q -y \
  libusb-dev \
  && rm -rf /var/lib/apt/lists/*

# Add non-privileged user
RUN groupadd -g 1000 swarmiedev
RUN useradd -M -N -g swarmiedev -d /SwarmiesRL -s /bin/bash swarmiedev

USER swarmiedev

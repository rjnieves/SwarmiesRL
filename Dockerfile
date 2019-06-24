FROM ros_kinetic_full-cudagl:latest

# Add non-privileged user
RUN groupadd -g 1000 swarmiedev
RUN useradd -M -N -g swarmiedev -d /SwarmiesRL -s /bin/bash swarmiedev

USER swarmiedev


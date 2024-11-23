# Base image
FROM osrf/ros:galactic-desktop-focal

# Install Gazebo 11 and other dependencies
RUN apt-get update && apt-get install -y \
  gazebo11 \
  ros-galactic-joint-state-publisher \
  ros-galactic-robot-state-publisher \
  ros-galactic-gazebo-plugins \
  ros-galactic-teleop-twist-keyboard \ 
  ros-galactic-xacro \
  ros-galactic-urdf \
  git \
  nano \
  python \
  && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy the files in the current directory into the container
COPY tortoisebot_gazebo /ros2_ws/src/tortoisebot_gazebo
COPY tortoisebot_description /ros2_ws/src/tortoisebot_description
COPY tortoisebot_bringup /ros2_ws/src/tortoisebot_bringup
COPY ydlidar-ros2 /ros2_ws/src/ydlidar-ros2
COPY tortoisebot_control /ros2_ws/src/tortoisebot_control
COPY tortoisebot_firmware /ros2_ws/src/tortoisebot_firmware
COPY tortoisebot_waypoints /ros2_ws/src/tortoisebot_waypoints

# Building the workspace
RUN /bin/bash -c "source /opt/ros/galactic/setup.bash && cd /ros2_ws && colcon build && source /ros2_ws/install/setup.bash"

# Source the workspace at startup
RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Starting a bash shell
CMD ["bash"]

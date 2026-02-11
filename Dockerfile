# Use the ROS 2 Humble base image
FROM ros:humble-ros-base

# Set shell to bash for ROS commands
SHELL ["/bin/bash", "-c"]

# Install system dependencies for the application
# - libsdl2-dev: For building against SDL2
# - libsdl2-2.0-0: For running SDL2 applications
# - ffmpeg & alsa-utils: For streaming
# - ros-humble-visualization-msgs: For the ROS message types
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libsdl2-dev \
    libsdl2-ttf-dev \
    libsdl2-2.0-0 \
    nlohmann-json3-dev \
    ffmpeg \
    alsa-utils \
    ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Create and set the working directory for the ROS workspace
WORKDIR /ros2_ws

# Copy the entire bob_sdlviz package into the workspace's src directory
# This assumes the build context is the bob_sdlviz directory
COPY . src/bob_sdlviz

# Source the ROS environment and build the package
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select bob_sdlviz --symlink-install

# Copy the entrypoint script and make it executable
COPY ./scripts/audio_bridge.py /usr/local/bin/audio_bridge.py
COPY ./scripts/start_stream.sh /usr/local/bin/start_stream.sh
RUN chmod +x /usr/local/bin/start_stream.sh /usr/local/bin/audio_bridge.py

# Set the entrypoint for the container
ENTRYPOINT ["/usr/local/bin/start_stream.sh"]
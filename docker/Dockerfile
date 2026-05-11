FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-rosgraph-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws

# Copy only the files needed to build the ROS adapter
COPY include/sim_schema.h                          /ros_ws/src/include/
COPY src/core/shm_backend.h                        /ros_ws/src/core/
COPY src/ros_adapter/ros_adapter.cpp               /ros_ws/src/ros_adapter/
COPY docker/CMakeLists_ros_adapter.txt             /ros_ws/src/CMakeLists.txt

# Build with colcon
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Entrypoint: run the adapter
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

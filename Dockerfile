FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential \
      lcov \
      python3-colcon-common-extensions \
      python3-pip \
      python3-pytest \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY src ./src
COPY docker ./docker

# COLCON_IGNORE files keep the retained ROS 1 crowd implementations out of
# discovery. rosdep installs the ROS 2 package dependencies from the manifests.
RUN rosdep update \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -r -y \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && colcon build --event-handlers console_direct+"

RUN chmod +x /workspace/docker/entrypoint.sh

ENTRYPOINT ["/workspace/docker/entrypoint.sh"]
CMD ["ros2", "launch", "semaforr", "example_simulation.launch.py"]


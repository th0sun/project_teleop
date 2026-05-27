FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    -o Dpkg::Options::="--force-confnew" \
    build-essential \
    cmake \
    git \
    fluxbox \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-utils \
    novnc \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pytest \
    ros-jazzy-ament-cmake-auto \
    ros-jazzy-ament-copyright \
    ros-jazzy-ament-flake8 \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-ament-pep257 \
    ros-jazzy-launch-ros \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-xacro \
    websockify \
    x11vnc \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

COPY tools/ros_jazzy/entrypoint.sh /usr/local/bin/project-teleop-jazzy
RUN chmod +x /usr/local/bin/project-teleop-jazzy

WORKDIR /project_teleop_ws/project_teleop
ENTRYPOINT ["/usr/local/bin/project-teleop-jazzy"]
CMD ["full"]

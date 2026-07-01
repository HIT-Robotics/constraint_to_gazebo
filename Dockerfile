
# ============================================================
# Stage 1: Base
# OS + ROS + system dependencies ONLY
# No users, no /home, no workspaces
# ============================================================

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO} AS base

ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get install -y \
    # General
    git \
    wget \
    sudo \
    gnupg2 \
    build-essential \
    cmake \
    python3-pip \
    python-is-python3 \
    python3-dev \

    # GUI / graphics
    x11-apps \
    mesa-utils \
    vainfo \
    mesa-va-drivers \
    libgl1 \
    libegl1 \
    libx11-6 \

    # Python stack
    python3-numpy \
    python3-scipy \
    python3-pandas \
    python3-matplotlib \
    python3-lxml \
    python3-yaml \
    python3-openpyxl \
    python3-tk \
    python3-pyqt5 \

    # C++ / robotics
    libeigen3-dev \
    libboost-all-dev \
    libtinyxml2-dev \
    libconsole-bridge-dev \
    pybind11-dev \

    # ROS tooling
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-eigenpy \
    ros-${ROS_DISTRO}-hpp-fcl \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Build limits (safe defaults)
ENV MAKEFLAGS="-j1"
ENV CMAKE_BUILD_PARALLEL_LEVEL=3
ENV COLCON_MAX_WORKERS=2

# gpu support for nvidia
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all


SHELL ["/bin/bash", "-lc"]

# ============================================================
# Stage 2: Devel
# Interactive development with volumes
# Uses non-root user
# ============================================================

FROM base AS devel

# Create robot user (DEV ONLY)
RUN useradd -m -s /bin/bash robot \
    && echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER robot

# X11 / GUI env
ENV DISPLAY=host.docker.internal:0.0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth
ENV XDG_RUNTIME_DIR=/home/robot/xdg_dir

RUN mkdir -p /home/robot/xdg_dir \
    && chmod 0700 /home/robot/xdg_dir

WORKDIR /home/robot/ws

# Interactive shell setup (user-specific is OK here)
RUN echo "export TERM=xterm-256color" >> ~/.bashrc \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "alias build='colcon build --symlink-install && source install/setup.bash'" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]

# ============================================================
# Stage 3: Builder
# Compiles the workspace
# Uses non-root user
# ============================================================

FROM base AS builder

# Create robot user (BUILD ONLY)
RUN useradd -m -s /bin/bash robot \
    && echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && mkdir -p /home/robot \
    && chown -R robot:robot /home/robot

USER robot
WORKDIR /base_ws

# Copy sources
COPY --chown=robot:robot src ./src

# Build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install

# ============================================================
# Stage 4: Runtime
# Minimal runtime image
# No users, no /home
# ============================================================

FROM base AS runtime

WORKDIR /base_ws

# Copy install artifacts only
COPY --from=builder /base_ws/install ./install

# Runtime ROS environment (global, user-agnostic)
ENV AMENT_PREFIX_PATH=/base_ws/install:/opt/ros/${ROS_DISTRO}
ENV CMAKE_PREFIX_PATH=/base_ws/install:/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/base_ws/install
ENV CMAKE_MODULE_PATH=/base_ws/install/share:/opt/ros/${ROS_DISTRO}/share:${CMAKE_MODULE_PATH}
ENV LD_LIBRARY_PATH=/base_ws/install/lib:$LD_LIBRARY_PATH
ENV PATH=/base_ws/install/bin:$PATH
ENV PYTHONPATH=/base_ws/install/lib/python3.12/site-packages:/base_ws/install/lib/python3/dist-packages:$PYTHONPATH

# Global bash sourcing for derived images
RUN echo "source /base_ws/install/setup.bash" > /etc/profile.d/base_ws_ros.sh

ENTRYPOINT ["/bin/bash"]

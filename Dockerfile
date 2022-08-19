# syntax=docker/dockerfile:1
ARG ROS_DISTRO="humble"
FROM osrf/ros:${ROS_DISTRO}-desktop
# Restate for later use
ARG ROS_DISTRO
ARG UIDGID
ARG USER
ARG REPO

# fail build if args are missing
RUN if [ -z "$UIDGID" ]; then echo '\nERROR: UIDGID not set. Run \n\n \texport UIDGID=$(id -u):$(id -g) \n\n on host before building Dockerfile.\n'; exit 1; fi
RUN if [ -z "$USER" ]; then echo '\nERROR: USER not set. Run \n\n \texport USER=$(whoami) \n\n on host before building Dockerfile.\n'; exit 1; fi

# prevent interactive messages in apt install
ARG DEBIAN_FRONTEND=noninteractive

# install development tools
RUN apt update \
    && apt install -q -y --no-install-recommends \
        apt-utils \
        ccache \
        clang-14 \
        clang-format \
        clang-tidy \
        cmake \
        git \
        lld \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pip \
        vim \
        wget \
        ssh-client \
        software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# install some pip packages needed for testing
RUN python3 -m pip install -U \
    pre-commit

# add github to known hosts
RUN mkdir -p -m 0700 ~/.ssh \
    && ssh-keyscan github.com >> ~/.ssh/known_hosts

# setup mixin and update it
COPY infrastructure/colcon-mixin /opt/colcon-mixin
RUN colcon mixin add ${REPO} file:///opt/colcon-mixin/index.yaml \
    && colcon mixin update

# build moveit_studio using it's upstream repos file
WORKDIR /opt/internal
RUN --mount=type=ssh \
    mkdir src \
    && git clone --single-branch --branch hackathon/remove-protobuf-requirement \
        git@github.com:PickNikRobotics/moveit_studio.git src/moveit_studio \
    && add-apt-repository ppa:openrobotics/gazebo11-non-amd64 \
    && vcs import src < src/moveit_studio/moveit_studio.repos \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update && apt update \
    && touch src/realsense-ros/realsense2_camera/COLCON_IGNORE \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --mixin release lld

# build source dependencies
WORKDIR /opt/upstream
COPY upstream.repos .

RUN --mount=type=ssh \
    mkdir src \
    && vcs import src < upstream.repos \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update && apt update \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --mixin release lld \
    && rm -rf build log src upstream.repos

# copy source to install repo dependencies
WORKDIR /ws
COPY . ./src/${REPO}
# install repo dependencies
RUN . /opt/upstream/install/setup.sh \
    && rosdep update && apt update \
    && rosdep install -q -y \
        --from-paths src \
        --ignore-src \
        --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt upgrade -y

# check that the repo builds and then clear it out
RUN . /opt/upstream/install/setup.sh \
    && colcon build --mixin release lld \
    && rm -rf /ws

# chown working directory to user
RUN mkdir -p /home/${USER}/ws && chown -R ${UIDGID} /home/${USER}

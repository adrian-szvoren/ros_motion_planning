# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /project

# RUN sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list
# Install additional dependencies if needed
RUN apt-get update \
    && apt-get -y --no-install-recommends install \
    git \
    gcc \
    vim \
    psmisc \
    libxml2-dev \
    libxslt-dev \
    python3 \
    python3-pip \ 
    python-is-python3\
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-navfn

# python packages
RUN pip3 install setuptools && pip3 install catkin-tools

# bash
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Copy the project into the container
COPY . /project

# Install osqp and osqposqp-eigen
RUN /project/scripts/install_osqp.sh

ENV CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH}:/project/packages/osqp/include"

# catkin build
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make;'

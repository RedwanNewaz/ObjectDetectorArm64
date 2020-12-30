FROM arm64v8/ros:kinetic-ros-base
  
ENV ROS_WS=/root/home
RUN mkdir -p $ROS_WS
WORKDIR $ROS_WS


RUN apt-get -qq update && \
    apt-get -qq install -y \
    apt-utils \
    ros-kinetic-uvc-camera &&\
    rm -rf /var/lib/apt/lists/*


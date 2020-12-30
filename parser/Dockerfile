FROM arm64v8/ros:noetic-perception

ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src/gem_joystick
WORKDIR $CATKIN_WS/src

COPY ./doods_parser ./doods_parser

# install build dependencies
RUN apt-get -qq update && apt install -y python3-pip


RUN apt-get -qq install -y \
        python3-catkin-tools 

RUN pip3 install requests osrf-pycommon

WORKDIR $CATKIN_WS

ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build
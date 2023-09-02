FROM ros:humble
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG PACKAGE=gpss_interfaces

WORKDIR $OVERLAY_WS/src
ADD . $PACKAGE

RUN apt-get update && \
    apt-get install -y \
    python3 \
    python3-pip \ 
    wget \
    ros-humble-xacro \
    ros-humble-joy \
    ros-humble-rviz2 \
    nlohmann-json3-dev \
    ros-humble-rmw-cyclonedds-cpp

WORKDIR $OVERLAY_WS
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --symlink-install \
    --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh


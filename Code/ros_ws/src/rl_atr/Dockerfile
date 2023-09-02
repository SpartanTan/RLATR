From gpss_atr:latest
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG PACKAGE=rl_atr

WORKDIR $OVERLAY_WS/src
ADD . $PACKAGE

WORKDIR $OVERLAY_WS
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
RUN pip install gymnasium
RUN . /opt/ros/humble/setup.sh && \ 
    colcon build \
    --symlink-install \
    --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    --packages-select atr_launch atr_rl_testpublisher

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh
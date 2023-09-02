FROM factory_db:latest
ARG OVERLAY_WS=/opt/ros/overlay_ws
ARG PACKAGE=gpss_atr

WORKDIR $OVERLAY_WS/src
ADD . $PACKAGE

WORKDIR $OVERLAY_WS
RUN . /opt/ros/humble/setup.sh && \ 
    colcon build \
    --symlink-install \
    --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
    --packages-select atr_utils && \
    colcon build \
    --symlink-install \
    --merge-install \
    --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=1

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh
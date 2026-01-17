FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"

USER root

# Define the argument with a default value of "true"
ARG BUILD=true

COPY ROS-TCP-Endpoint ./src/ROS-TCP-Endpoint
RUN catkin build ros_tcp_endpoint

COPY ubt_msgs ./src/ubt_msgs
RUN catkin build ubt_msgs

COPY orb_slam3_ros ./src/orb_slam3_ros
RUN catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1

# Use shell logic to conditionally run the build
# If BUILD is "true", it builds; otherwise, it echoes a message and skips.
RUN if [ "$BUILD" = "true" ] ; then \
        catkin build orb_slam3_ros; \
    else \
        echo "Skipping build of orb_slam3_ros"; \
    fi

COPY serenade ./src/serenade
COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY bridge.yaml /bridge.yaml
COPY init.sh /init.sh

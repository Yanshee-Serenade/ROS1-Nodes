FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"

USER root

COPY ROS-TCP-Endpoint ./src/ROS-TCP-Endpoint
RUN catkin build ros_tcp_endpoint

COPY ubt_msgs ./src/ubt_msgs
RUN catkin build ubt_msgs

COPY orb_slam3_ros ./src/orb_slam3_ros
RUN catkin build orb_slam3_ros

RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc
COPY serenade ./src/serenade
COPY ros_entrypoint.sh /ros_entrypoint.sh
COPY bridge.yaml /bridge.yaml
COPY start_bridge.launch /start_bridge.launch
COPY init.sh /init.sh

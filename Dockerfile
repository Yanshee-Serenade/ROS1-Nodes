FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"

USER root

COPY ROS-TCP-Endpoint ./src/ROS-TCP-Endpoint
RUN catkin build ros_tcp_endpoint

COPY orb_slam3_ros ./src/orb_slam3_ros
RUN catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
RUN catkin build orb_slam3_ros

COPY ubt_msgs ./src/ubt_msgs
RUN catkin build ubt_msgs

COPY serenade ./src/serenade
COPY entrypoint.sh /entrypoint.sh
COPY init.sh /init.sh

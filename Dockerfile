FROM lucasmogsan/orbslam3_ros:latest
LABEL maintainer="subcat2077@gmail.com"

USER root

COPY ROS-TCP-Endpoint ./src/ROS-TCP-Endpoint
RUN catkin build ros_tcp_endpoint

COPY orb_slam3_ros ./src/orb_slam3_ros
RUN catkin build orb_slam3_ros

# TODO migrate these
COPY decompress_node.py /decompress_node.py
COPY imu_node.py /imu_node.py
COPY image_server.py /image_server.py
COPY angle_server.py /angle_server.py
COPY param_server.py /param_server.py
COPY pose_server.py /pose_server.py
COPY entrypoint.sh /entrypoint.sh
COPY orbslam3_init.sh /orbslam3_init.sh
COPY ubt_msgs_py ./devel/lib/python3/dist-packages/ubt_msgs
COPY ubt_msgs ./devel/share/ubt_msgs

python /overlay_ws/src/serenade/decompress_node.py &
python /overlay_ws/src/serenade/imu_node.py &
python /overlay_ws/src/serenade/image_server.py &
python /overlay_ws/src/serenade/angle_server.py &
python /overlay_ws/src/serenade/param_server.py &
python /overlay_ws/src/serenade/pose_server.py &
roscore &
roslaunch --wait ros_tcp_endpoint endpoint.launch &
roslaunch --wait orb_slam3_ros pi.launch

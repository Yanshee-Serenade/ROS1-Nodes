python /overlay_ws/src/serenade/decompress_node.py &
python /overlay_ws/src/serenade/imu_node.py &
python /overlay_ws/src/serenade/camera_info_node.py &
python /overlay_ws/src/serenade/angle_server.py &
roslaunch /start_bridge.launch

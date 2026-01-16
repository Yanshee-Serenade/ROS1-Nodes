python /overlay_ws/src/yanshee/decompress_node.py &
python /overlay_ws/src/yanshee/imu_node.py &
python /overlay_ws/src/yanshee/image_server.py &
python /overlay_ws/src/yanshee/angle_server.py &
python /overlay_ws/src/yanshee/param_server.py &
python /overlay_ws/src/yanshee/pose_server.py &
roscore &
roslaunch --wait ros_tcp_endpoint endpoint.launch &
roslaunch --wait orb_slam3_ros pi.launch

python /overlay_ws/src/serenade/decompress_node.py &
python /overlay_ws/src/serenade/imu_node.py &
python /overlay_ws/src/serenade/camera_info_node.py &
python /overlay_ws/src/serenade/image_server.py &
python /overlay_ws/src/serenade/angle_server.py &
python /overlay_ws/src/serenade/param_server.py &
python /overlay_ws/src/serenade/pose_server.py &
roscore &
sleep 5
rosparam load /bridge.yaml
roslaunch ros_tcp_endpoint endpoint.launch &
roslaunch orb_slam3_ros pi.launch

cmake_minimum_required(VERSION 3.0.2)
project(ubt_msgs)

# 1. Add geometry_msgs to find_package
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  action_report.msg
  angles_set.msg
  april_tag_detection_array.msg
  april_tag_detection.msg
  asr_iat_data.msg
  battery_report.msg
  face_attribute.msg
  face_data.msg
  gamepad_key.msg
  gamepad.msg
  gyro_report.msg
  layer.msg
  motion_planning_speed.msg
  sensor_report.msg
  StandaloneTag.msg
  tts_data.msg
)

add_service_files(
  DIRECTORY srv
)

# 2. Add geometry_msgs to generate_messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 3. Add geometry_msgs to catkin_package
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)

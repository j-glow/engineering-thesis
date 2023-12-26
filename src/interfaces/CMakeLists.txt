cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/DetectionBox.msg"
)

set(srv_files
  "srv/Inference.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime sensor_msgs)

ament_package()

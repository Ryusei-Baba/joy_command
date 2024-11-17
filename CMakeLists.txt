cmake_minimum_required(VERSION 3.8)
project(joy_command)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(joy_command_node src/joy_command_node.cpp)

ament_target_dependencies(joy_command_node
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS
  joy_command_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

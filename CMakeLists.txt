cmake_minimum_required(VERSION 3.8)
project(joy_command_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(${PROJECT_NAME} src/joy_command_node.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_srvs
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

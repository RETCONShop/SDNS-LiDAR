cmake_minimum_required(VERSION 3.8)
project(snl_movementcontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snl_messages REQUIRED)
find_package(irobot_create_msgs REQUIRED)

add_executable(snl_movementcontrol
  src/snl_movementcontrol.cpp
)
include_directories(include)
ament_target_dependencies(snl_movementcontrol rclcpp irobot_create_msgs snl_messages geometry_msgs)
install(TARGETS
  snl_movementcontrol
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

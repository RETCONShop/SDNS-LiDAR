cmake_minimum_required(VERSION 3.8)
project(snl_commandin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snl_messages REQUIRED)
find_package(irobot_create_msgs REQUIRED)

add_executable(snl_commandin
  src/snl_commandin.cpp
)
ament_target_dependencies(snl_commandin rclcpp irobot_create_msgs snl_messages)
install(TARGETS
  snl_commandin
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(snl_kalmanfilter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snl_messages REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(snl_kalmanfilter
  src/snl_kalmanfilter.cpp
)

target_include_directories(snl_kalmanfilter PUBLIC "${Eigen3_INCLUDE_DIRS}")
ament_target_dependencies(snl_kalmanfilter rclcpp irobot_create_msgs snl_messages eigen3_cmake_module)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
install(TARGETS
snl_kalmanfilter
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

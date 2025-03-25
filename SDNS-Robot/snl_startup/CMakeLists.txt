cmake_minimum_required(VERSION 3.8)
project(snl_startup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snl_messages REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(snl_startup
src/snl_startup.cpp
)
ament_target_dependencies(snl_startup rclcpp irobot_create_msgs snl_messages)
install(TARGETS
snl_startup
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

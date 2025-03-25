cmake_minimum_required(VERSION 3.8)
project(snl_vn200)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-ignored-qualifiers -Wno-unused-value -Wno-unused-variable)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snl_messages REQUIRED)
find_package(irobot_create_msgs REQUIRED)

add_subdirectory(cpp)
include_directories(cpp/include)

add_executable(snl_vn200
  src/snl_vn200.cpp
)
ament_target_dependencies(snl_vn200 rclcpp irobot_create_msgs snl_messages)
target_link_libraries(snl_vn200 libvncxx)
install(TARGETS
  snl_vn200
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

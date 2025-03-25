cmake_minimum_required(VERSION 3.8)
project(snl_lidarcollector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-ignored-qualifiers -Wno-unused-value)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(snl_messages REQUIRED)
include_directories( sdk/include sdk/src)
add_executable(snl_lidarnode
src/snl_lidarnode.cpp
sdk/src/sl_lidar_driver.cpp
sdk/src/sl_lidarprotocol_codec.cpp
sdk/src/sl_serial_channel.cpp
sdk/src/sl_udp_channel.cpp
sdk/src/sl_crc.cpp
sdk/src/dataunpacker/dataunpacker.cpp
sdk/src/dataunpacker/unpacker/handler_capsules.cpp
sdk/src/dataunpacker/unpacker/handler_hqnode.cpp
sdk/src/dataunpacker/unpacker/handler_normalnode.cpp
sdk/src/sl_async_transceiver.cpp
sdk/src/hal/thread.cpp
sdk/src/arch/linux/net_serial.cpp
sdk/src/arch/linux/net_socket.cpp
sdk/src/arch/linux/timer.cpp
)
ament_target_dependencies(snl_lidarnode rclcpp std_msgs snl_messages)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  snl_lidarnode
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

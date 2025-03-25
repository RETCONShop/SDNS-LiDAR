cmake_minimum_required(VERSION 3.8)
project(snl_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/Lidar.msg"
"msg/LidarPoint.msg"
"msg/Odom.msg"
"msg/Imu.msg"
"msg/MechImu.msg"
"msg/NavOut.msg"
"msg/NavCmd.msg"
"msg/FilterOutput.msg"
"msg/MechMeasurement.msg"
DEPENDENCIES std_msgs geometry_msgs
)
ament_package()

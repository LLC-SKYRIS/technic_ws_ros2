cmake_minimum_required(VERSION 3.16)
project(vl53l1x C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MeasurementData.msg
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# STM VL53L1X API library, unchanged from the ROS 1 driver.
file(GLOB vl53l1x_core lib/core/*.c)
file(GLOB vl53l1x_platform lib/platform/*.c)

add_executable(vl53l1x_node
  src/vl53l1x.cpp
  src/i2c.cpp
  ${vl53l1x_core}
  ${vl53l1x_platform}
)
target_include_directories(vl53l1x_node PRIVATE src lib/core lib/platform)
target_link_libraries(vl53l1x_node
  ${cpp_typesupport_target}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS vl53l1x_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()

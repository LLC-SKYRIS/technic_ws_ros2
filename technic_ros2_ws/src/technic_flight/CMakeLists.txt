cmake_minimum_required(VERSION 3.16)
project(technic_flight)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(technic_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(technic_px4_backend
  src/px4_backend.cpp
  src/mavros_backend.cpp
)
target_include_directories(technic_px4_backend
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(technic_px4_backend
  geometry_msgs
  mavros_msgs
  px4_msgs
  rclcpp
  sensor_msgs
  tf2
)

add_executable(flight_manager
  src/flight_manager.cpp
  src/frame_resolver.cpp
)
target_link_libraries(flight_manager technic_px4_backend)
ament_target_dependencies(flight_manager
  geometry_msgs
  rclcpp
  rclcpp_action
  std_msgs
  technic_interfaces
  tf2
  tf2_ros
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS technic_px4_backend flight_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(mavros_msgs px4_msgs rclcpp)
ament_package()

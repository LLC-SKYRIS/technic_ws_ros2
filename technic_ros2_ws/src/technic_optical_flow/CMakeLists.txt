cmake_minimum_required(VERSION 3.16)
project(technic_optical_flow)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d)

add_library(${PROJECT_NAME}_component SHARED src/optical_flow.cpp)
target_link_libraries(${PROJECT_NAME}_component ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_component
  cv_bridge
  geometry_msgs
  mavros_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "technic_optical_flow::OpticalFlow"
  EXECUTABLE optical_flow
)

install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

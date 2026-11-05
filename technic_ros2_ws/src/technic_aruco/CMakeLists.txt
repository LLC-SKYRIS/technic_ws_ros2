cmake_minimum_required(VERSION 3.16)
project(technic_aruco)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(technic_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Point2D.msg"
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  "srv/SetMarkers.srv"
  DEPENDENCIES geometry_msgs std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# ArUco module vendored from the ROS 1 aruco_pose package: keeps the detector
# API and behaviour independent of the system OpenCV version (see VendorOpenCV.cmake).
add_library(_opencv_aruco STATIC
  vendor/aruco/src/apriltag_quad_thresh.cpp
  vendor/aruco/src/aruco.cpp
  vendor/aruco/src/charuco.cpp
  vendor/aruco/src/dictionary.cpp
  vendor/aruco/src/zmaxheap.cpp
)
target_include_directories(_opencv_aruco BEFORE PUBLIC vendor/aruco/include)
target_include_directories(_opencv_aruco PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(_opencv_aruco PUBLIC ${OpenCV_LIBRARIES})
target_compile_definitions(_opencv_aruco PRIVATE CV_OVERRIDE=override)
target_compile_options(_opencv_aruco PRIVATE -fPIC -fvisibility=hidden)

add_library(${PROJECT_NAME}_components SHARED
  src/aruco_detect.cpp
  src/aruco_map.cpp
  src/draw.cpp
  src/vpe_publisher.cpp
)
# The vendored aruco headers must win over a system opencv2/aruco.hpp.
target_include_directories(${PROJECT_NAME}_components BEFORE PRIVATE vendor/aruco/include)
target_link_libraries(${PROJECT_NAME}_components
  _opencv_aruco
  "${cpp_typesupport_target}"
)
ament_target_dependencies(${PROJECT_NAME}_components
  cv_bridge
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  technic_interfaces
  tf2
  tf2_ros
  visualization_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "technic_aruco::ArucoDetect"
  EXECUTABLE aruco_detect
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "technic_aruco::ArucoMap"
  EXECUTABLE aruco_map
)
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "technic_aruco::VpePublisher"
  EXECUTABLE vpe_publisher
)

install(
  TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch map DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE vendor/)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.16)
project(technic_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ROS 1 compatible services used by the web pages (see scripts/web_api).
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Execute.srv"
  "srv/GenMap.srv"
  "srv/GetTelemetry.srv"
  "srv/Navigate.srv"
  "srv/SetVelocity.srv"
  "srv/SetYawRate.srv"
  # blocks programming backend (ROS 1 technic_blocks interfaces)
  "msg/Prompt.msg"
  "srv/Run.srv"
  "srv/Load.srv"
  "srv/Store.srv"
)

# Program API used by the blocks programs (technic_blocks.api, technic_blocks.gpio).
ament_python_install_package(technic_blocks)

install(PROGRAMS
  scripts/web_server
  scripts/web_api
  scripts/bag_to_mp4
  scripts/butterfly_terminal
  scripts/technic_blocks
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch www DESTINATION share/${PROJECT_NAME})
# blocks page (served at /technic_blocks/) and example programs
install(DIRECTORY blocks/www blocks/programs DESTINATION share/${PROJECT_NAME}/technic_blocks)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

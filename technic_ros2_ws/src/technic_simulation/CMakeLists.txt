cmake_minimum_required(VERSION 3.16)
project(technic_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/smoke_mission.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME smoke_mission
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

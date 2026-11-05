cmake_minimum_required(VERSION 3.8)
project(technic_docs NONE)

find_package(ament_cmake REQUIRED)

# Markdown sources are rendered in the browser by docs/_viewer (no build step).
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)

ament_package()

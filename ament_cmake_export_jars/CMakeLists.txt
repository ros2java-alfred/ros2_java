cmake_minimum_required(VERSION 3.5)

project(ament_cmake_export_jars NONE)

find_package(ament_cmake_core REQUIRED)

set(ament_cmake_export_jars_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

ament_package(
  CONFIG_EXTRAS "ament_cmake_export_jars-extras.cmake"
    "ament_cmake_package_templates-extras.cmake"
    "cmake/ament_java_package_hook.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

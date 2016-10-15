cmake_minimum_required(VERSION 2.8.3)

project(rcljava_common)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_export_jars REQUIRED)
find_package(ament_cmake_export_libraries REQUIRED)

if(ANDROID)
  find_host_package(Java COMPONENTS Development REQUIRED)
else()
  find_package(Java COMPONENTS Development REQUIRED)
  find_package(JNI REQUIRED)
endif()
include(UseJava)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

set(CMAKE_JAVA_COMPILE_FLAGS "-source" "1.6" "-target" "1.6")

set(${PROJECT_NAME}_java_sources
  "src/main/java/org/ros2/rcljava/common/RCLJavaProxy.java"
  "src/main/java/org/ros2/rcljava/internal/message/Message.java"
  "src/main/java/org/ros2/rcljava/internal/service/Service.java"
)

set(${PROJECT_NAME}_cpp_sources
  "src/main/cpp/rcljava_common.cpp"
)

add_jar("${PROJECT_NAME}_jar"
  "${${PROJECT_NAME}_java_sources}"
  OUTPUT_NAME
  "${PROJECT_NAME}"
)

install_jar("${PROJECT_NAME}_jar" "share/${PROJECT_NAME}/java")
ament_export_jars("share/${PROJECT_NAME}/java/${PROJECT_NAME}.jar")

include_directories(include)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_cpp_sources})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${JNI_INCLUDE_DIRS}
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rcljava_cmake_module-extras.cmake.in"
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

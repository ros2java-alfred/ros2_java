cmake_minimum_required(VERSION 2.8.3)

project(rcljava_examples)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_jars REQUIRED)
find_package(rcljava REQUIRED)
find_package(std_msgs REQUIRED)

if (ANDROID)
  find_host_package(Java COMPONENTS Development)
else()
  find_package(Java COMPONENTS Development)
endif()
include (UseJava)

if (NOT ANDROID)
  find_package(JNI REQUIRED)
endif()

set(${PROJECT_NAME}_sources
  "src/Talker.java"
)

add_jar("${PROJECT_NAME}_jar"
  "${${PROJECT_NAME}_sources}"
  OUTPUT_NAME
  "${PROJECT_NAME}"
  INCLUDE_JARS
  "${rcljava_JARS}"
  "${std_msgs_JARS}"
)

install_jar("${PROJECT_NAME}_jar" "share/java")

ament_package()

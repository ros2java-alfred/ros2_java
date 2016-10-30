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

find_jar(SFL4J_JAR NAMES slf4j-api)
if(NOT ${SFL4J_JAR})
  set(log4j_version "1.2.17")
  set(slf4j_version "1.7.21")

  set(log4j_sha256 "1d31696445697720527091754369082a6651bd49781b6005deb94e56753406f9")
  set(slf4j_api_sha256 "1d5aeb6bd98b0fdd151269eae941c05f6468a791ea0f1e68d8e7fe518af3e7df")
  set(slf4j_log4j_sha256 "9563e26fd7863cca0f5d468ef09f2927047f8b85a8c76d98766ed5863cb678b2")
  set(slf4j_android_sha256 "3874b0975f9ad283328c0371ccde2de353754da33fda36e611702623ee1e5a95")
  set(slf4j_jdk14_sha256 "2c369503c911bf9ac7c5f4813bc1f8f95866e2e029fae203a02712f5671f5e4a")

  set(log4j_url "http://central.maven.org/maven2/log4j/log4j/${log4j_version}/log4j-${log4j_version}.jar")
  set(slf4j_api_url "http://central.maven.org/maven2/org/slf4j/slf4j-api/${slf4j_version}/slf4j-api-${slf4j_version}.jar")
  set(slf4j_log4j_url "http://central.maven.org/maven2/org/slf4j/slf4j-log4j12/${slf4j_version}/slf4j-log4j12-${slf4j_version}.jar")
  set(slf4j_android_url "http://central.maven.org/maven2/org/slf4j/slf4j-android/${slf4j_version}/slf4j-android-${slf4j_version}.jar")
  set(slf4j_jdk14_url "http://central.maven.org/maven2/org/slf4j/slf4j-jdk14/${slf4j_version}/slf4j-jdk14-${slf4j_version}.jar")

  set(log4j_jar_path "${CMAKE_CURRENT_BINARY_DIR}/jars/log4j-${log4j_version}.jar")
  set(slf4j_api_jar_path "${CMAKE_CURRENT_BINARY_DIR}/jars/slf4j-api-${slf4j_version}.jar")
  set(slf4j_log4j_jar_path "${CMAKE_CURRENT_BINARY_DIR}/jars/slf4j-log4j12-${slf4j_version}.jar")
  set(slf4j_android_jar_path "${CMAKE_CURRENT_BINARY_DIR}/jars/slf4j-android-${slf4j_version}.jar")
  set(slf4j_jdk14_jar_path "${CMAKE_CURRENT_BINARY_DIR}/jars/slf4j-jdk14-${slf4j_version}.jar")

  file(DOWNLOAD ${log4j_url} ${log4j_jar_path} EXPECTED_HASH SHA256=${log4j_sha256})
  file(DOWNLOAD ${slf4j_api_url} ${slf4j_api_jar_path} EXPECTED_HASH SHA256=${slf4j_api_sha256})
  file(DOWNLOAD ${slf4j_log4j_url} ${slf4j_log4j_jar_path} EXPECTED_HASH SHA256=${slf4j_log4j_sha256})
  file(DOWNLOAD ${slf4j_android_url} ${slf4j_android_jar_path} EXPECTED_HASH SHA256=${slf4j_android_sha256})
  file(DOWNLOAD ${slf4j_jdk14_url} ${slf4j_jdk14_jar_path} EXPECTED_HASH SHA256=${slf4j_jdk14_sha256})

  install(FILES
    ${log4j_jar_path} ${slf4j_api_jar_path}
    ${slf4j_log4j_jar_path} ${slf4j_android_jar_path}
    ${slf4j_jdk14_jar_path}
    DESTINATION
    "share/${PROJECT_NAME}/java")

  ament_export_jars("share/${PROJECT_NAME}/java/log4j-${log4j_version}.jar")
  ament_export_jars("share/${PROJECT_NAME}/java/slf4j-api-${slf4j_version}.jar")
  ament_export_jars("share/${PROJECT_NAME}/java/slf4j-log4j12-${slf4j_version}.jar")
  ament_export_jars("share/${PROJECT_NAME}/java/slf4j-android-${slf4j_version}.jar")
  ament_export_jars("share/${PROJECT_NAME}/java/slf4j-jdk14-${slf4j_version}.jar")
endif()

ament_export_jars(${SFL4J_JAR})

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
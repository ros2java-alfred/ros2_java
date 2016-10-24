cmake_minimum_required(VERSION 2.8.3)

project(rcljava)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_jars REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rcljava_common REQUIRED)
find_package(JavaExtra MODULE)

if(ANDROID)
  find_host_package(Java COMPONENTS Development)
else()
  find_package(Java COMPONENTS Development)
  find_package(JNI REQUIRED)
endif()

include(UseJava)

set(CMAKE_JAVA_COMPILE_FLAGS "-source" "1.6" "-target" "1.6")

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

include_directories(include)

macro(target)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  endif()

  set(_jni_classes "RCLJava;Node;Publisher;Client")

  foreach(_jni_class ${_jni_classes})

    set(_target_name "${PROJECT_NAME}${_jni_class}${target_suffix}")

    add_library(
      ${_target_name}
      SHARED src/main/cpp/org_ros2_rcljava_${_jni_class}.cpp
    )
    target_compile_definitions(${_target_name}
      PRIVATE "RMW_IMPLEMENTATION_SUFFIX=${target_suffix}")

    if(WIN32)
      set(_build_types ";_DEBUG;_MINSIZEREL;_RELEASE;_RELWITHDEBINFO")
    else()
      set(_build_types "")
    endif()

    foreach(_build_type ${_build_types})
      set_target_properties(${_target_name} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
        RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
      )
    endforeach()

    ament_target_dependencies(${_target_name}
      "rcl${target_suffix}"
      "rcljava_common"
    )

    target_include_directories(${_target_name}
      PUBLIC
      ${JNI_INCLUDE_DIRS}
    )

    ament_export_libraries(${_target_name})

    install(TARGETS ${_target_name}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
    )

  endforeach()

endmacro()

set(${PROJECT_NAME}_sources
  "src/main/java/org/ros2/rcljava/BiConsumer.java"
  "src/main/java/org/ros2/rcljava/Client.java"
  "src/main/java/org/ros2/rcljava/Consumer.java"
  "src/main/java/org/ros2/rcljava/Node.java"
  "src/main/java/org/ros2/rcljava/package-info.java"
  "src/main/java/org/ros2/rcljava/Publisher.java"
  "src/main/java/org/ros2/rcljava/qos/policies/Durability.java"
  "src/main/java/org/ros2/rcljava/qos/policies/History.java"
  "src/main/java/org/ros2/rcljava/qos/policies/QoSPolicy.java"
  "src/main/java/org/ros2/rcljava/qos/policies/Reliability.java"
  "src/main/java/org/ros2/rcljava/qos/QoSProfile.java"
  "src/main/java/org/ros2/rcljava/RCLFuture.java"
  "src/main/java/org/ros2/rcljava/RCLJava.java"
  "src/main/java/org/ros2/rcljava/RMWRequestId.java"
  "src/main/java/org/ros2/rcljava/Service.java"
  "src/main/java/org/ros2/rcljava/Subscription.java"
  "src/main/java/org/ros2/rcljava/TriConsumer.java"
)

add_jar("${PROJECT_NAME}_jar"
  "${${PROJECT_NAME}_sources}"
  OUTPUT_NAME
  "${PROJECT_NAME}"
  INCLUDE_JARS
  "${rcljava_common_JARS}"
)

call_for_each_rmw_implementation(target)

set(_install_jar_dir "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
get_property(_jar_file TARGET "${PROJECT_NAME}_jar" PROPERTY "JAR_FILE")

install_jar("${PROJECT_NAME}_jar" "share/${PROJECT_NAME}/java")
ament_export_jars("share/${PROJECT_NAME}/java/${PROJECT_NAME}.jar")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(${PROJECT_NAME}_test_sources
    "src/test/java/org/ros2/rcljava/NodeTest.java"
    "src/test/java/org/ros2/rcljava/PublisherTest.java"
    "src/test/java/org/ros2/rcljava/RCLJavaTest.java"
    "src/test/java/org/ros2/rcljava/SubscriptionTest.java"
  )

  set(${PROJECT_NAME}_testsuites
    "org.ros2.rcljava.NodeTest"
    "org.ros2.rcljava.PublisherTest"
    "org.ros2.rcljava.RCLJavaTest"
    "org.ros2.rcljava.SubscriptionTest"
  )

  set(_deps_library_paths "")
  foreach(_dep_lib ${std_msgs_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  list(APPEND _deps_library_paths ${CMAKE_CURRENT_BINARY_DIR})

  foreach(testsuite ${${PROJECT_NAME}_testsuites})
  add_junit_tests("${PROJECT_NAME}_tests_${testsuite}"
    "${${PROJECT_NAME}_test_sources}"
    TESTS
    "${testsuite}"
    INCLUDE_JARS
    "${rcljava_common_JARS}"
    "${std_msgs_JARS}"
    "${_jar_file}"
    LIBRARY_PATHS
    "${_deps_library_paths}"
  )
  endforeach()

endif()

ament_package()

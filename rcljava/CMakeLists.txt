cmake_minimum_required(VERSION 3.5)

project(rcljava)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_jars REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(rcljava_common REQUIRED)

find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs)
find_package(sensor_msgs)

include(JavaExtra)

enable_testing()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Default Flags
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include_directories(include)

find_jar(JUNIT_JAR              NAMES junit                   PATHS /usr/share/java VERSIONS 4)
find_jar(LOG_JAR                NAMES commons-logging         PATHS /usr/share/java)
find_jar(STD_MSGS_JARS          NAMES std_msgs                PATHS "${CMAKE_BINARY_DIR}/../std_msgs/")
find_jar(GEOMETRY_MSGS_JARS     NAMES geometry_msgs           PATHS "${CMAKE_BINARY_DIR}/../geometry_msgs/")
find_jar(SENSOR_MSGS_JARS       NAMES sensor_msgs             PATHS "${CMAKE_BINARY_DIR}/../sensor_msgs/")
find_jar(RCLINTERFACE_MSGS_JARS NAMES rcl_interfaces          PATHS "${CMAKE_BINARY_DIR}/../rcl_interfaces/")
find_jar(BUILTIN_MSGS_JARS      NAMES builtin_interfaces      PATHS "${CMAKE_BINARY_DIR}/../builtin_interfaces/")
find_jar(BUILTIN_COMMON_JARS    NAMES rosidl_generator_java   PATHS "${CMAKE_BINARY_DIR}/../rosjava_common/")

set(STD_MSGS_JARS                 "${CMAKE_BINARY_DIR}/../std_msgs/std_msgs.jar")
set(GEOMETRY_MSGS_JARS            "${CMAKE_BINARY_DIR}/../geometry_msgs/geometry_msgs.jar")
set(SENSOR_MSGS_JARS              "${CMAKE_BINARY_DIR}/../sensor_msgs/sensor_msgs.jar")
set(BUILTIN_MSGS_JARS             "${CMAKE_BINARY_DIR}/../builtin_interfaces/builtin_interfaces.jar")
set(RCLINTERFACE_MSGS_JARS  NAMES "${CMAKE_BINARY_DIR}/../rcl_interfaces/rcl_interfaces.jar")
set(BUILTIN_COMMON_JARS     NAMES "${CMAKE_BINARY_DIR}/../rosjava_common/rosjava_common.jar")

#set(CMAKE_CURRENT_SOURCE_DIR "src/main/java/")

set(${PROJECT_NAME}_sources_java
  "src/main/java/org/ros2/rcljava/RCLJava.java"
  "src/main/java/org/ros2/rcljava/Logger.java"
  "src/main/java/org/ros2/rcljava/ArgumentParser.java"

  "src/main/java/org/ros2/rcljava/internal/NativeUtils.java"

  "src/main/java/org/ros2/rcljava/namespace/GraphName.java"

  "src/main/java/org/ros2/rcljava/qos/policies/Durability.java"
  "src/main/java/org/ros2/rcljava/qos/policies/History.java"
  "src/main/java/org/ros2/rcljava/qos/policies/QoSPolicy.java"
  "src/main/java/org/ros2/rcljava/qos/policies/Reliability.java"
  "src/main/java/org/ros2/rcljava/qos/QoSProfile.java"

  # WallTimer Stack Base
  "src/main/java/org/ros2/rcljava/time/WallTimer.java"
  "src/main/java/org/ros2/rcljava/time/WallTimerCallback.java"
  "src/main/java/org/ros2/rcljava/time/BaseWallTimer.java"
  # WallTimer Implementation
  "src/main/java/org/ros2/rcljava/time/NativeWallTimer.java"
  "src/main/java/org/ros2/rcljava/time/JavaWallTimer.java"

  # Node Stack Base
  "src/main/java/org/ros2/rcljava/node/internal/NodeClock.java"
  "src/main/java/org/ros2/rcljava/node/internal/NodeGraph.java"
  "src/main/java/org/ros2/rcljava/node/internal/NodeLogging.java"
  "src/main/java/org/ros2/rcljava/node/internal/NodeParameters.java"
  "src/main/java/org/ros2/rcljava/node/internal/NodeServices.java"
  "src/main/java/org/ros2/rcljava/node/internal/NodeTimers.java"
  "src/main/java/org/ros2/rcljava/node/internal/NodeTopics.java"
  "src/main/java/org/ros2/rcljava/node/Node.java"
  "src/main/java/org/ros2/rcljava/node/BaseNode.java"
  # Node Implementation
  "src/main/java/org/ros2/rcljava/node/NativeNode.java"
  "src/main/java/org/ros2/rcljava/node/JavaNode.java"

  "src/main/java/org/ros2/rcljava/node/topic/Topics.java"
  # Topic Publisher Stack Base
  "src/main/java/org/ros2/rcljava/node/topic/Publisher.java"
  "src/main/java/org/ros2/rcljava/node/topic/BasePublisher.java"
  # Topic Publisher Implementation
  "src/main/java/org/ros2/rcljava/node/topic/NativePublisher.java"
  "src/main/java/org/ros2/rcljava/node/topic/JavaPublisher.java"

  # Topic Subscription Stack Base
  "src/main/java/org/ros2/rcljava/node/topic/Subscription.java"
  "src/main/java/org/ros2/rcljava/node/topic/SubscriptionCallback.java"
  "src/main/java/org/ros2/rcljava/node/topic/BaseSubscription.java"
  # Topic Publisher Implementation
  "src/main/java/org/ros2/rcljava/node/topic/NativeSubscription.java"
  "src/main/java/org/ros2/rcljava/node/topic/JavaSubscription.java"

  "src/main/java/org/ros2/rcljava/node/service/NativeServiceType.java"
  # Client Stack Base
  "src/main/java/org/ros2/rcljava/node/service/Client.java"
  "src/main/java/org/ros2/rcljava/node/service/BaseClient.java"
  # Client Implementation
  "src/main/java/org/ros2/rcljava/node/service/NativeClient.java"
  "src/main/java/org/ros2/rcljava/node/service/JavaClient.java"

  # Service Stack Base
  "src/main/java/org/ros2/rcljava/node/service/Service.java"
  "src/main/java/org/ros2/rcljava/node/service/ServiceCallback.java"
  "src/main/java/org/ros2/rcljava/node/service/BaseService.java"
  # Service Implementation
  "src/main/java/org/ros2/rcljava/node/service/NativeService.java"
  "src/main/java/org/ros2/rcljava/node/service/JavaService.java"

  "src/main/java/org/ros2/rcljava/node/service/RCLFuture.java"
  "src/main/java/org/ros2/rcljava/node/service/RMWRequestId.java"

  "src/main/java/org/ros2/rcljava/node/parameter/ParameterCallback.java"
  "src/main/java/org/ros2/rcljava/node/parameter/ParameterEventCallback.java"
  "src/main/java/org/ros2/rcljava/node/parameter/ParameterService.java"
  "src/main/java/org/ros2/rcljava/node/parameter/ParameterVariant.java"
  "src/main/java/org/ros2/rcljava/node/parameter/SyncParametersClient.java"

  "src/main/java/org/ros2/rcljava/executor/AnyExecutable.java"
  "src/main/java/org/ros2/rcljava/executor/BaseThreadedExecutor.java"
  "src/main/java/org/ros2/rcljava/executor/DefaultThreadedExecutor.java"
  "src/main/java/org/ros2/rcljava/executor/MemoryStrategy.java"
  "src/main/java/org/ros2/rcljava/executor/MultiThreadedExecutor.java"
  "src/main/java/org/ros2/rcljava/executor/NativeExecutor.java"
  "src/main/java/org/ros2/rcljava/executor/SingleThreadedExecutor.java"
  "src/main/java/org/ros2/rcljava/executor/ThreadedExecutor.java"

  "src/main/java/org/ros2/rcljava/exception/ImplementationAlreadyImportedException.java"
  "src/main/java/org/ros2/rcljava/exception/InvalidRCLJAVAImplementation.java"
  "src/main/java/org/ros2/rcljava/exception/NoImplementationAvailableException.java"
  "src/main/java/org/ros2/rcljava/exception/NotImplementedException.java"
  "src/main/java/org/ros2/rcljava/exception/NotInitializedException.java"
)

set(${PROJECT_NAME}_sources_cpp
  "src/main/cpp/org_ros2_rcljava_RCLJava.cpp"
  "src/main/cpp/org_ros2_rcljava_executor_NativeExecutor.cpp"
  "src/main/cpp/org_ros2_rcljava_node_NativeNode.cpp"
  "src/main/cpp/org_ros2_rcljava_node_topic_NativePublisher.cpp"
  "src/main/cpp/org_ros2_rcljava_node_topic_NativeSubscription.cpp"
  "src/main/cpp/org_ros2_rcljava_node_service_NativeClient.cpp"
  "src/main/cpp/org_ros2_rcljava_node_service_NativeService.cpp"
  "src/main/cpp/org_ros2_rcljava_time_NativeWallTimer.cpp"
  "src/main/cpp/utils.cpp"
)

set(${PROJECT_NAME}_test_sources
  "src/test/java/org/ros2/rcljava/AbstractRosTest.java"
  "src/test/java/org/ros2/rcljava/AbstractMessageTest.java"

  "src/test/java/org/ros2/rcljava/executor/ExecutorTest.java"
  "src/test/java/org/ros2/rcljava/executor/ExecutorFailTest.java"
  "src/test/java/org/ros2/rcljava/namespace/GraphNameTest.java"
  "src/test/java/org/ros2/rcljava/node/NodeTest.java"
  "src/test/java/org/ros2/rcljava/node/topic/PublisherTest.java"
  "src/test/java/org/ros2/rcljava/node/topic/SubscriptionTest.java"
  "src/test/java/org/ros2/rcljava/node/service/ServiceTest.java"
  "src/test/java/org/ros2/rcljava/time/TimerTest.java"

  "src/test/java/org/ros2/rcljava/MessageStdTest.java"
  "src/test/java/org/ros2/rcljava/MessageSensorTest.java"
  "src/test/java/org/ros2/rcljava/MessageGeometryTest.java"

  "src/test/java/org/ros2/rcljava/RCLJavaTest.java"
  "src/test/java/org/ros2/rcljava/RCLJavaFailTest.java"
  "src/test/java/org/ros2/rcljava/RmwTest.java"
)

set(${PROJECT_NAME}_testsuites
  "org.ros2.rcljava.executor.ExecutorTest"
  "org.ros2.rcljava.executor.ExecutorFailTest"
  "org.ros2.rcljava.namespace.GraphNameTest"
  "org.ros2.rcljava.node.NodeTest"
  "org.ros2.rcljava.node.topic.PublisherTest"
  "org.ros2.rcljava.node.topic.SubscriptionTest"
  "org.ros2.rcljava.node.service.ServiceTest"
  "org.ros2.rcljava.time.TimerTest"

  "org.ros2.rcljava.MessageStdTest"
  "org.ros2.rcljava.MessageSensorTest"
  "org.ros2.rcljava.MessageGeometryTest"

  "org.ros2.rcljava.RCLJavaTest"
  "org.ros2.rcljava.RCLJavaFailTest"
  "org.ros2.rcljava.RmwTest"
)

get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE "c")
if(typesupport_impls STREQUAL "")
  message(STATUS "Skipping rcljava because no C typesupport library was found.")
  return()
endif()

###
# Compile rcljava.so
###

set(_target_name "${PROJECT_NAME}${target_suffix}")

add_library(
  ${_target_name}
  SHARED
    "${${PROJECT_NAME}_sources_cpp}"
)

target_compile_definitions(
  ${_target_name}
  PRIVATE
    "RMW_IMPLEMENTATION_SUFFIX=${target_suffix}"
)

if(WIN32)
  set(_build_types ";_DEBUG;_MINSIZEREL;_RELEASE;_RELWITHDEBINFO")
else()
  set(_build_types "")
endif()

foreach(_build_type ${_build_types})
  set_target_properties(
    ${_target_name}
    PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
      RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}"
  )
endforeach()

ament_target_dependencies(${_target_name}
  "rcl${target_suffix}"
  "rcljava_common"
  "rcl_interfaces"
)

target_include_directories(
  ${_target_name}
  PUBLIC
    ${JNI_INCLUDE_DIRS}
)

#  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    ament_export_libraries(${_target_name})
#  endif()

install(TARGETS ${_target_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


###
# Compile rcljava.jar
###

set(CMAKE_JAVA_INCLUDE_PATH "${BUILTIN_COMMON_JARS}:${BUILTIN_MSGS_JARS}:${LOG_JAR}")
add_jar("${PROJECT_NAME}_jar"
  "${${PROJECT_NAME}_sources_java}"
  OUTPUT_NAME
  "${PROJECT_NAME}"
  INCLUDE_JARS
  "${std_msgs_JARS}"
  "${builtin_interfaces_JARS}"
  "${rcl_interfaces_JARS}"
  "${rcljava_common_JARS}"
  "${_jar_file}"
)

#call_for_each_rmw_implementation(target)

install_jar("${PROJECT_NAME}_jar" "share/${PROJECT_NAME}/java")
ament_export_jars("share/${PROJECT_NAME}/java/${PROJECT_NAME}.jar")
ament_java_package_hook(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  get_property(_${PROJECT_NAME}_jar_file
    TARGET "${PROJECT_NAME}_jar"
    PROPERTY "JAR_FILE")

  set(_deps_library_paths "")
  foreach(_dep_lib ${rcljava_test_msgs_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  foreach(_dep_lib ${std_msgs_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  foreach(_dep_lib ${geometry_msgs_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  foreach(_dep_lib ${sensor_msgs_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  foreach(_dep_lib ${rcl_interfaces_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  foreach(_dep_lib ${builtin_interfaces_LIBRARIES})
    get_filename_component(_dep_dir "${_dep_lib}" DIRECTORY)
    list(APPEND _deps_library_paths ${_dep_dir})
  endforeach()
  list(APPEND _deps_library_paths ${CMAKE_CURRENT_BINARY_DIR})

  list(REMOVE_DUPLICATES _deps_library_paths)
  list(SORT _deps_library_paths)
  list(APPEND _deps_library_paths ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_java/rcljava/msg/)

  foreach(testsuite ${${PROJECT_NAME}_testsuites})
    add_junit_tests("${PROJECT_NAME}_tests_${testsuite}"
      "${${PROJECT_NAME}_test_sources}"
      TESTS
      "${testsuite}"
      INCLUDE_JARS
      "${rcljava_common_JARS}"
      "${rcljava_test_msgs_JARS}"
      "${std_msgs_JARS}"
      "${geometry_msgs_JARS}"
      "${sensor_msgs_JARS}"
      "${builtin_interfaces_JARS}"
      "${rcl_interfaces_JARS}"
      "${_${PROJECT_NAME}_jar_file}"
      "${_jar_file}"
      LIBRARY_PATHS
      "${_deps_library_paths}"
    )
  endforeach()

#  create_javadoc("${PROJECT_NAME}_docs"
#    FILES "${${PROJECT_NAME}_sources_java}"
#    SOURCEPATH "src/main/java/"
#    CLASSPATH
#    "${CMAKE_JAVA_INCLUDE_PATH}"
#    "${rcljava_common_JARS}"
#    "${rcljava_test_msgs_JARS}"
#    "${std_msgs_JARS}"
#    "${geometry_msgs_JARS}"
#    "${sensor_msgs_JARS}"
#    "${builtin_interfaces_JARS}"
#    "${rcl_interfaces_JARS}"
#    "${_${PROJECT_NAME}_jar_file}"
#    "${_jar_file}"
#    INSTALLPATH "docs"
#    WINDOWTITLE "ROS2 Java"
#    DOCTITLE "<h1>ROS2 Java</h1>"
#    AUTHOR TRUE
#    USE TRUE
#    VERSION TRUE
#  )

#  execute_process(COMMAND "javasphinx-apidoc -o ../docs/source/ --title='ROS2 for Java' src/main/java/")

endif()

ament_package()

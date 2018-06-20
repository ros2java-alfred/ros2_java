cmake_minimum_required(VERSION 3.5)

project(rosidl_generator_java)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rcljava_common REQUIRED)

include(JavaExtra)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rmw)
ament_export_dependencies(rosidl_cmake)

ament_index_register_resource("rosidl_generator_packages")

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)

  find_package(rmw REQUIRED)
  find_package(rosidl_cmake REQUIRED)
  find_package(rosidl_generator_c REQUIRED)
  
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/Bool.msg"
    "msg/Byte.msg"
    "msg/Char.msg"
    "msg/Constants.msg"
    "msg/Empty.msg"
    "msg/Float32.msg"
    "msg/Float64.msg"
    "msg/Int16.msg"
    "msg/Int32.msg"
    "msg/Int64.msg"
    "msg/Int8.msg"
    "msg/Nested.msg"
    "msg/Primitives.msg"
    "msg/StringArrays.msg"
    "msg/Strings.msg"
    "msg/Uint16.msg"
    "msg/Uint32.msg"
    "msg/Uint64.msg"
    "msg/Uint8.msg"
    "msg/Various.msg"
  )

  include(cmake/register_java.cmake)
  include(cmake/rosidl_generator_java_get_typesupports.cmake)

  set(rosidl_generator_java_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  # Need to call extras before get_typesupports, to register the extension
  rosidl_generator_java_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_java"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_java/__init__.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  # If no message files were generated, skip the test.
  rosidl_generator_java_get_typesupports(_typesupport_impls)
  if(_typesupport_impls STREQUAL "")
    message(WARNING "No valid typesupport found for Java generator, skipping tests.")
  else()
    rosidl_generate_interfaces(${PROJECT_NAME} ${message_files}
      ADD_LINTER_TESTS
      SKIP_INSTALL
    )

    set(${PROJECT_NAME}_test_sources
      "src/test/java/org/ros2/generator/InterfacesTest.java"
    )

    set(${PROJECT_NAME}_testsuites
      "org.ros2.generator.InterfacesTest"
    )

    get_property(_${PROJECT_NAME}_messages_jar_file
      TARGET "${PROJECT_NAME}_messages_jar"
      PROPERTY "JAR_FILE")

    set(_deps_library_dirs "")
    list_append_unique(_deps_library_dirs ${CMAKE_CURRENT_BINARY_DIR})
    list_append_unique(_deps_library_dirs ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_java/rosidl_generator_java/msg/)

    foreach(testsuite ${${PROJECT_NAME}_testsuites})
      add_junit_tests("${PROJECT_NAME}_tests_${testsuite}"
        TIMEOUT 90
        WORKING_DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}"
        SOURCES
        "${${PROJECT_NAME}_test_sources}"
        TESTS
        "${testsuite}"
        INCLUDE_JARS
        "${rcljava_common_JARS}"
        "${rosidl_generator_java_JARS}"
        "${_${PROJECT_NAME}_messages_jar_file}"
      )

      add_dependencies("${PROJECT_NAME}_tests_${testsuite}" "${PROJECT_NAME}_messages_jar")
    endforeach()
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "cmake/rosidl_generator_java_get_typesupports.cmake"
    "cmake/register_java.cmake"
    "rosidl_generator_java-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_java
  DESTINATION lib/rosidl_generator_java
)

install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)

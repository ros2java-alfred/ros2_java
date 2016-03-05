cmake_minimum_required(VERSION 2.8.3)

project(rcljava)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_jars REQUIRED)
find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

find_package(Java COMPONENTS Development)
include (UseJava)

if (NOT ANDROID)
  find_package(JNI REQUIRED)
endif()

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

# enables using the Python package from the source space
# This creates a folder in ${CMAKE_CURRENT_BINARY_DIR}/rcljava which has all the
# as well as the compiled modules from the build space
# necessary Python code and C Python extensions for running tests.

#ament_python_install_package(${PROJECT_NAME})

macro(target)
  if(NOT "${target_suffix} " STREQUAL " ")
    get_rcl_information("${rmw_implementation}" "rcl${target_suffix}")
  endif()
  if("${rmw_implementation} " STREQUAL "rmw_connext_dynamic_cpp ")
    message(STATUS "Skipping tests for '${rmw_implementation}'")
    return()
  endif()
  if("${rmw_implementation} " STREQUAL "rmw_fastrtps_cpp ")
    message(STATUS "Skipping tests for '${rmw_implementation}'")
    return()
  endif()

  set(_jni_classes "RCLJava;Node;Publisher")
  #set(_jni_classes "Node")

  foreach(_jni_class ${_jni_classes})

    set(_target_name "${PROJECT_NAME}${_jni_class}${target_suffix}")

    add_library(
      ${_target_name}
      SHARED src/main/cpp/io_ebinoma_rcljava_${_jni_class}.cpp
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
    )

    target_include_directories(${_target_name}
      PUBLIC
      ${JNI_INCLUDE_DIRS}
    )

    if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
      ament_export_libraries(${_target_name})

      install(TARGETS ${_msg_name}${_javaext_suffix}
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${_msg_package_dir2}"
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
      )
      install(TARGETS ${_target_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
      )
    endif()

  endforeach()

endmacro()

set(${PROJECT_NAME}_sources
  "src/main/java/io/ebinoma/rcljava/Node.java"
  "src/main/java/io/ebinoma/rcljava/RCLJava.java"
  "src/main/java/io/ebinoma/rcljava/Publisher.java"
)

add_jar("${PROJECT_NAME}_jar"
  "${${PROJECT_NAME}_sources}"
  OUTPUT_NAME
  "${PROJECT_NAME}"
)

call_for_each_rmw_implementation(target)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_nose_test(rcljavatests test
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  set(_install_jar_dir "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
  get_property(_jar_file TARGET "${PROJECT_NAME}_jar" PROPERTY "JAR_FILE")

  install_jar("${PROJECT_NAME}_jar" "share/java")
  ament_export_jars("share/java/${PROJECT_NAME}.jar")
endif()

ament_package()

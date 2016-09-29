# Copyright 2016 Esteve Fernandez <esteve@apache.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if(ANDROID)
  find_host_package(Java COMPONENTS Development)
else()
  find_package(Java COMPONENTS Development)
  find_package(JNI REQUIRED)
endif()

include(UseJava)

function(add_junit_tests TARGET_NAME)

  cmake_parse_arguments(_add_junit_tests
    ""
    ""
    "TESTS;SOURCES;INCLUDE_JARS;LIBRARY_PATHS"
    ${ARGN}
  )

  set(_source_files ${_add_junit_tests_SOURCES} ${_add_junit_tests_UNPARSED_ARGUMENTS})

  if(WIN32 AND NOT CYGWIN)
    set(SEPARATOR ";")
  else()
    set(SEPARATOR ":")
  endif()

  find_jar(JUNIT_JAR NAMES junit4)
  if(NOT ${JUNIT_JAR})
    find_jar(JUNIT_JAR NAMES junit VERSIONS 4)
  endif()

  add_jar("${TARGET_NAME}_jar"
    "${_source_files}"
    OUTPUT_NAME
    "${TARGET_NAME}"
    INCLUDE_JARS
    "${_add_junit_tests_INCLUDE_JARS}"
    "${JUNIT_JAR}"
  )

  get_property(_jar_test_file
    TARGET "${TARGET_NAME}_jar"
    PROPERTY "JAR_FILE"
  )

  set(${TARGET_NAME}_jar_dependencies "${JUNIT_JAR}${SEPARATOR}${_jar_test_file}")
  foreach(_jar_dep ${_add_junit_tests_INCLUDE_JARS})
    set(${TARGET_NAME}_jar_dependencies "${${TARGET_NAME}_jar_dependencies}${SEPARATOR}${_jar_dep}")
  endforeach()

  string(REPLACE ";" ${SEPARATOR} _library_paths "${_add_junit_tests_LIBRARY_PATHS}")

  add_test(NAME ${TARGET_NAME}
    COMMAND ${Java_JAVA_EXECUTABLE}
    ${JVMARGS} -classpath ${${TARGET_NAME}_jar_dependencies} -Djava.library.path=${_library_paths}
    org.junit.runner.JUnitCore ${_add_junit_tests_TESTS}
  )

#  add_dependencies(${TARGET_NAME} ${TARGET_NAME}_jar)
endfunction()

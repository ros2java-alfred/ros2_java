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

# Based on https://goo.gl/UsQN1c
# Used a URL shortener to avoid complaints from link_cmake

# macro to find packages on the host OS
macro( find_host_package )
  set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER )
  set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER )
  set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER )
  if( CMAKE_HOST_WIN32 )
    set( WIN32 1 )
    set( UNIX )
  elseif( CMAKE_HOST_APPLE )
    set( APPLE 1 )
    set( UNIX )
  endif()
  find_package( ${ARGN} )
  set( WIN32 )
  set( APPLE )
  set( UNIX 1 )
  set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY )
  set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY )
  set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY )
endmacro()

# macro to find programs on the host OS
macro( find_host_program )
  set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER )
  set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER )
  set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER )
  if( CMAKE_HOST_WIN32 )
    set( WIN32 1 )
    set( UNIX )
  elseif( CMAKE_HOST_APPLE )
    set( APPLE 1 )
    set( UNIX )
  endif()
  find_program( ${ARGN} )
  set( WIN32 )
  set( APPLE )
  set( UNIX 1 )
  set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY )
  set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY )
  set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY )
endmacro()

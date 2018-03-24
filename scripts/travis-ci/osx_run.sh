#!/usr/bin/env sh
# scripts/travis-ci/osx_run.sh

set -e

cd $ROS2WS
. $ROS2WS/install_isolated/local_setup.sh
ament test --symlink-install --isolated --only-packages ament_cmake_export_jars rcljava rcljava_common rosidl_generator_java

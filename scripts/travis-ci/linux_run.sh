#!/bin/bash
# scripts/travis-ci/linux_run.sh


set -ev

cd $HOME_BUILD
docker run -u "$UID" -it --rm -v $(pwd):$(pwd) --env-file $ENV_PATH -w $(pwd) $DOCKER_REPO:$DOCKER_DIST sh -c ". $HOME_BUILD/ament_ws/install/local_setup.sh && cd /home/travis/build/ros2_java_ws && . ./install/local_setup.sh && colcon test --packages-select ament_cmake_export_jars rcljava rcljava_common"

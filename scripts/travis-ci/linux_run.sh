#!/usr/bin/env sh
# scripts/travis-ci/linux_run.sh


set -ev

cd $HOME_BUILD
docker run -u "$UID" -it --rm -v $(pwd):$(pwd) --env-file $ENV_PATH -w $(pwd) $DOCKER_REPO:$DOCKER_DIST sh -c ". ament_ws/install_isolated/local_setup.sh && cd /home/travis/build/ros2_java_ws && . ./install_isolated/local_setup.sh && ament test --symlink-install --isolated --only-packages ament_cmake_export_jars rcljava rcljava_common"

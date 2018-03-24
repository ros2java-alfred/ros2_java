#!/bin/sh
# scripts/travis-ci/linux_success.sh

set -ev

if [ ${TRAVIS_BRANCH} == "master" ]; then
    docker run -u "$UID" -it --env-file $ENV_PATH -w $(pwd) --name="artefact" $DOCKER_REPO:$DOCKER_DIST sh -c "echo ok"
    docker cp -L $HOME_BUILD/. artefact:$HOME_BUILD
    docker commit -m "Build $TRAVIS_BUILD_NUMBER" artefact $DOCKER_REPO:$DOCKER_DIST-ros2java
    docker push $DOCKER_REPO:$DOCKER_DIST-ros2java

    if [ ${TRAVIS_EVENT_TYPE} != "cron" ]; then
        docker tag $DOCKER_REPO:$DOCKER_DIST-ros2java $DOCKER_REPO:$DOCKER_DIST-ros2java_$TRAVIS_BUILD_NUMBER ;
    fi

    if [ ${TRAVIS_EVENT_TYPE} != "cron" ]; then
        docker push $DOCKER_REPO:$DOCKER_DIST-ros2java_$TRAVIS_BUILD_NUMBER ;
    fi
fi

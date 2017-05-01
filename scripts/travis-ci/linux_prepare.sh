# scripts/travis-ci/linux_prepare.sh
#!/bin/sh

set -ev

DOCKER_IMG="$DOCKER_REPO:$DOCKER_DIST"

# Docker Hub authenticate.
docker login -e="$DOCKER_EMAIL" -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
docker pull $DOCKER_IMG

# Make shared environment variables.
cd $HOME_BUILD
env | grep -E '^TRAVIS_' > $ENV_PATH && \
env | grep -E '^COVERALLS_' >> $ENV_PATH && \
env | grep -E '^CI_' >> $ENV_PATH && \
echo -e "CI_BUILD_NUMBER=$TRAVIS_BUILD_NUMBER\nCI_PULL_REQUEST=$TRAVIS_PULL_REQUEST\nCI_BRANCH=$TRAVIS_BRANCH\nCI_BUILD_URL=" >> $ENV_PATH

# Check container variables.
docker run -u "$UID" -it --rm -v `pwd`:`pwd` --env-file $ENV_PATH -w `pwd` $DOCKER_IMG sh -c "locale && env | grep -E '^TRAVIS_' && env | grep -E '^CI_' && env | grep -E '^JAVA_'"

echo "INSTALL/BUILD ROS2 AMENT..."
mkdir -p $HOME_BUILD/ament_ws/src
cd $HOME_BUILD/ament_ws
docker run -u "$UID" -it --rm -v `pwd`:`pwd` -w `pwd` $DOCKER_IMG sh -c "/usr/bin/wget https://gist.githubusercontent.com/Theosakamg/e6084cfafa6b7ea690104424cef970a2/raw/ament_java.repos"
docker run -u "$UID" -it --rm -v `pwd`:`pwd` -w `pwd` $DOCKER_IMG sh -c "/usr/bin/vcs import src < ament_java.repos"
docker run -u "$UID" -it --rm -v `pwd`:`pwd` -w `pwd` $DOCKER_IMG sh -c "src/ament/ament_tools/scripts/ament.py build --symlink-install --isolated"

echo "INSTALL ROS2 WS..."
mkdir -p $ROS2WS/src
cd $ROS2WS
docker run -u "$UID" -it --rm -v `pwd`:`pwd` -w `pwd` $DOCKER_IMG sh -c "/usr/bin/wget https://gist.githubusercontent.com/Theosakamg/d9259bbc708c5145255fbdeb25e65e19/raw/ros2_java_desktop.repos"
docker run -u "$UID" -it --rm -v `pwd`:`pwd` -w `pwd` $DOCKER_IMG sh -c "/usr/bin/vcs import src < ros2_java_desktop.repos"

# Patch for Java support.
cd $ROS2WS/src/ros2/rosidl_typesupport && patch -p1 < ../../ros2_java/ros2_java/rosidl_ros2_java.diff

# Fix eProsima dependancies.
cd $ROS2WS/src/eProsima/Fast-RTPS && git submodule init && git submodule update

# Sync with git trigger
rm -rf $ROS2WS/src/ros2_java/ros2_java && ln -s $HOME_BUILD/ros2java-alfred/ros2_java  $ROS2WS/src/ros2_java/ros2_java

echo "BUILD ROS2 WS..."
cd $HOME_BUILD
docker run -u "$UID" -it --rm -v `pwd`:`pwd` --env-file $ENV_PATH -w `pwd` $DOCKER_IMG sh -c ". ament_ws/install_isolated/local_setup.sh && cd /home/travis/build/ros2_java_ws && ament build --symlink-install --isolated --skip-packages $PKG_EXCLUDE --ament-gradle-args --parallel --daemon --configure-on-demand"

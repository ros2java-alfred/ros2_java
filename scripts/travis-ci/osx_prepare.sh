# scripts/travis-ci/osx_prepare.sh
#!/bin/sh

set -e

# Install default OSX stack.
time brew update
time brew tap osrf/ros2
time brew tap ros/deps
time brew install python3 wget cppcheck gtest tinyxml eigen pcre # cmake already installed
time brew install asio tinyxml2
#brew install Caskroom/cask/java
time brew install gradle
time brew tap homebrew/science
#  - brew install opencv --without-python
time sudo -H python3 -m pip install empy setuptools nose vcstool pep8 pydocstyle pyflakes flake8 mock coverage
gem install xcpretty

echo "INSTALL/BUILD ROS2 AMENT..."
mkdir -p $HOME_BUILD/ament_ws/src
cd $HOME_BUILD/ament_ws
wget https://gist.githubusercontent.com/Theosakamg/e6084cfafa6b7ea690104424cef970a2/raw/ament_java.repos
vcs import $HOME_BUILD/ament_ws/src < ament_java.repos
time src/ament/ament_tools/scripts/ament.py build --parallel --symlink-install --isolated

echo "INSTALL ROS2 WS..."
mkdir -p $ROS2WS/src
cd $ROS2WS
wget https://gist.githubusercontent.com/Theosakamg/d9259bbc708c5145255fbdeb25e65e19/raw/ros2_java_desktop.repos
vcs import $ROS2WS/src < ros2_java_desktop.repos

# Sync with git trigger
rm -rf $ROS2WS/src/ros2_java/ros2_java && ln -s $HOME/build/ros2java-alfred/ros2_java  $ROS2WS/src/ros2_java/ros2_java

# Patch for Java support.
cd $ROS2WS/src/ros2/rosidl_typesupport && patch -p1 < ../../ros2_java/ros2_java/rosidl_ros2_android.diff

echo "BUILD ROS2 WS..."
cd $HOME_BUILD
. ./ament_ws/install_isolated/local_setup.sh
cd $ROS2WS
ament build --parallel --symlink-install --isolated --skip-packages $PKG_EXCLUDE --ament-gradle-args --parallel --daemon --configure-on-demand

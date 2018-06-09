#!/bin/bash
# scripts/travis-ci/osx_prepare.sh

set -e

# Install default OSX stack.
brew update
brew tap osrf/ros2
brew tap ros/deps
brew upgrade python
brew install cppcheck tinyxml eigen pcre # wget cmake  python3 already installed
brew install asio tinyxml2
brew install gradle
brew tap caskroom/versions
brew cask install java8
#brew tap homebrew/science
#  - brew install opencv --without-python
sudo -H python3 -m pip install argcomplete catkin_pkg coverage empy flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes mock nose pep8 pydocstyle pyflakes pyparsing pytest pytest-cov pytest-runner pyyaml vcstool
# setuptools already installed
gem install xcpretty

export JAVA_HOME=/Library/Java/JavaVirtualMachines/jdk1.8.0_172.jdk/Contents/Home/

echo "INSTALL/BUILD ROS2 AMENT..."
mkdir -p $HOME_BUILD/ament_ws/src
cd $HOME_BUILD/ament_ws
wget https://gist.githubusercontent.com/Theosakamg/e6084cfafa6b7ea690104424cef970a2/raw/ament_java.repos
vcs import $HOME_BUILD/ament_ws/src < ament_java.repos
src/ament/ament_tools/scripts/ament.py build --parallel --symlink-install --isolated

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

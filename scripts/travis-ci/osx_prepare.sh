# scripts/travis-ci/osx_prepare.sh
#!/bin/sh

set -ev

# Install default OSX stack.
brew tap osrf/ros2
brew tap ros/deps
brew install python3 wget cppcheck gtest tinyxml eigen pcre cmake # cmake already installed
brew install asio tinyxml2
brew install Caskroom/cask/java
brew install gradle
brew tap homebrew/science
#  - brew install opencv --without-python
sudo -H python3 -m pip install empy setuptools nose vcstool pep8 pydocstyle pyflakes flake8 mock coverage
gem install xcpretty

echo "INSTALL/BUILD ROS2 AMENT..."
mkdir -p $HOME_BUILD/ament_ws/src
cd $HOME_BUILD/ament_ws
wget https://gist.githubusercontent.com/Theosakamg/e6084cfafa6b7ea690104424cef970a2/raw/ament_java.repos
vcs import $HOME_BUILD/ament_ws/src < ament_java.repos
src/ament/ament_tools/scripts/ament.py build --parallel --symlink-install --isolated

echo "INSTALL ROS2 WS..."
mkdir -p $ROS2WS/src
cd $ROS2WS
wget https://gist.githubusercontent.com/Theosakamg/259bbc708c5145255fbdeb25e65e19/raw/ros2_java_desktop.repos
vcs import $ROS2WS/src < ros2_java_desktop_travis.repos

# Patch for Java support.
cd $ROS2WS/src/ros2/rosidl_typesupport && patch -p1 < ../../ros2_java/ros2_java/rosidl_ros2_java.diff

# Sync with git trigger
rm -rf $ROS2WS/src/ros2_java/ros2_java && ln -s ~/ros2java-alfred/ros2_java  $ROS2WS/src/ros2_java/ros2_java

echo "BUILD ROS2 WS..."
cd $ROS2WS
. $HOME_BUILD/ament_ws/install_isolated/local_setup.sh
ament build --parallel --symlink-install --isolated --skip-packages $PKG_EXCLUDE --ament-gradle-args --parallel --daemon --configure-on-demand

@echo off
REM scripts/appveyor/windows_prepare.bat

CD %HOME%

ECHO. & ECHO Download components...
curl -skL https://github.com/ros2/choco-packages/releases/download/2017-04-04-1/asio.1.10.6.nupkg -o asio.1.10.6.nupkg
curl -skL https://github.com/ros2/choco-packages/releases/download/2017-04-04-1/eigen.3.3.3.nupkg -o eigen.3.3.3.nupkg
curl -skL https://github.com/ros2/choco-packages/releases/download/2017-04-04-1/tinyxml-usestl.2.6.2.nupkg -o tinyxml-usestl.2.6.2.nupkg
curl -skL https://github.com/ros2/choco-packages/releases/download/2017-04-04-1/tinyxml2.4.0.1.nupkg -o tinyxml2.4.0.1.nupkg

ECHO. & ECHO Install VCS tools...
pip install vcstool
python.exe -m pip install -U setuptools pip
python.exe -m pip install catkin_pkg EmPy pyparsing pyyaml
python.exe -m pip install nose coverage mock pytest pytest-cov pytest-runner
python.exe -m pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pep8 pydocstyle pyflakes


ECHO. & ECHO Install components...
choco install -y -s %HOME% asio eigen tinyxml-usestl tinyxml2

ECHO. & ECHO Check version...
git.exe --version
python.exe --version
cmake.exe --version

ECHO. & ECHO INSTALL/BUILD ROS2 AMENT...
MKDIR "%HOME_BUILD%\ament_ws\src"
CD %HOME_BUILD%\ament_ws
wget https://gist.githubusercontent.com/Theosakamg/e6084cfafa6b7ea690104424cef970a2/raw/ament_java.repos
vcs import %HOME_BUILD%\ament_ws\src < ament_java.repos
python.exe src\ament\ament_tools\scripts\ament.py build --parallel
REM --symlink-install --isolated

ECHO. & ECHO INSTALL ROS2 WS...
MKDIR "%ROS2WS%\src"
CD %ROS2WS%
wget https://gist.githubusercontent.com/Theosakamg/d9259bbc708c5145255fbdeb25e65e19/raw/ros2_java_desktop.repos
vcs import %ROS2WS%\src < ros2_java_desktop.repos

ECHO. & ECHO Sync with git trigger
RMDIR /Q /S %ROS2WS%\src\ros2_java\ros2_java && MKLINK /D %ROS2WS%\src\ros2_java\ros2_java %HOME_REPO%

ECHO. & ECHO BUILD ROS2 WS...
CD %HOME_BUILD%
CALL "%HOME_BUILD%\ament_ws\install\setup.bat"
CD %ROS2WS%
ament build --skip-packages %PKG_EXCLUDE% --parallel --ament-gradle-args --parallel --daemon --configure-on-demand
REM --symlink-install --isolated 





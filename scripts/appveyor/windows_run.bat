@echo off
REM scripts/appveyor/windows_run.bat

ECHO. & ECHO TEST APP...
CD %HOME_BUILD%
CALL "%HOME_BUILD%\ament_ws\install\setup.bat"
CD %ROS2WS%
CALL "%ROS2WS%\install\setup.bat"
ament test --only-packages ament_cmake_export_jars rcljava rcljava_common
REM --symlink-install --isolated 

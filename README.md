ROS2 for Java
=============

What is this?
-------------

This is a set of projects (bindings, code generator, examples and more) that enables developers to write ROS2
applications for the JVM and Android.

Besides this repository itself, there's also:
- https://github.com/esteve/ament_java, which adds support for Gradle to Ament
- https://github.com/esteve/ament_gradle_plugin, a Gradle plugin that makes it easier to use ROS2 in Java and Android project, and which can be found published at the Gradle Central at https://plugins.gradle.org/plugin/org.ros2.tools.gradle
- https://github.com/esteve/ros2_java_examples, examples for the Java Runtime Environment
- https://github.com/esteve/ros2_android_examples, examples for Android

Is this Java only?
------------------

No, any language that targets the JVM can be used to write ROS2 applications.

Including Android?
------------------

Yep! Make sure to use [this fork](https://github.com/eProsima/Fast-RTPS/pull/26) as your DDS vendor.

Features
--------

The current set of features include:
- Generation of all builtin and complex ROS types, including arrays, strings, nested types, constants, etc.
- Support for publishers and subscriptions
- Tunable Quality of Service (e.g. lossy networks, reliable delivery, etc.)
- Clients and services
- Support for Android

Sounds great, how can I try this out?
-------------------------------------

First of all, download the ament repositories in a separate workspace:

```
mkdir -p ~/ament_ws/src
cd ~/ament_ws
wget https://raw.githubusercontent.com/esteve/ament_java/master/ament_java.repos
vcs import ~/ament_ws/src < ament_java.repos
src/ament/ament_tools/scripts/ament.py build --symlink-install --isolated
```

You may wonder why this is needed if the ROS2 instructions already fetch Ament on the same workspace as ROS2.

The reason is that this includes an additional build type for Gradle projects, and you'll need Ament to pick it up so it can build the examples, so this has to happen in a separate step.

The following sections deal with building the `ros2_java` codebase for the desktop Java runtime and for Android.

Desktop
-------

```
mkdir -p ~/ros2_java_ws/src
cd ~/ros2_java_ws
wget https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_desktop.repos
vcs import ~/ros2_java_ws/src < ros2_java_desktop.repos
cd ~/ros2_java_ws/src/ros2/rosidl
patch -p1 < ../../ros2_java/ros2_java/rosidl_ros2_java.diff
cd ~/ros2_java_ws
. ~/ament_ws/install_isolated/local_setup.sh
ament build --symlink-install --isolated
```

Now you can just run a couple of examples.

Talker and Listener
-------------------

Talker:

```
. ~/ament_ws/install_isolated/local_setup.sh
. ~/ros2_java_ws/install_isolated/local_setup.sh
cd ~/ros2_java_ws
java -cp install_isolated/std_msgs/share/std_msgs/java/std_msgs.jar:install_isolated/rcljava/share/rcljava/java/rcljava.jar:install_isolated/rcljava_examples/share/rcljava_examples/java/rcljava_examples.jar:install_isolated/example_interfaces/share/example_interfaces/java/example_interfaces.jar:install_isolated/rcljava_common/share/rcljava_common/java/rcljava_common.jar org.ros2.rcljava.examples.Talker
```

Listener:

```
. ~/ament_ws/install_isolated/local_setup.sh
. ~/ros2_java_ws/install_isolated/local_setup.sh
cd ~/ros2_java_ws
java -cp install_isolated/std_msgs/share/std_msgs/java/std_msgs.jar:install_isolated/rcljava/share/rcljava/java/rcljava.jar:install_isolated/rcljava_examples/share/rcljava_examples/java/rcljava_examples.jar:install_isolated/example_interfaces/share/example_interfaces/java/example_interfaces.jar:install_isolated/rcljava_common/share/rcljava_common/java/rcljava_common.jar org.ros2.rcljava.examples.Listener
```

Client and Service
------------------

Client:

```
. ~/ament_ws/install_isolated/local_setup.sh
. ~/ros2_java_ws/install_isolated/local_setup.sh
cd ~/ros2_java_ws
java -cp install_isolated/std_msgs/share/std_msgs/java/std_msgs.jar:install_isolated/rcljava/share/rcljava/java/rcljava.jar:install_isolated/rcljava_examples/share/rcljava_examples/java/rcljava_examples.jar:install_isolated/example_interfaces/share/example_interfaces/java/example_interfaces.jar:install_isolated/rcljava_common/share/rcljava_common/java/rcljava_common.jar org.ros2.rcljava.examples.AddTwoIntsClient
```

Service:

```
. ~/ament_ws/install_isolated/local_setup.sh
. ~/ros2_java_ws/install_isolated/local_setup.sh
cd ~/ros2_java_ws
java -cp install_isolated/std_msgs/share/std_msgs/java/std_msgs.jar:install_isolated/rcljava/share/rcljava/java/rcljava.jar:install_isolated/rcljava_examples/share/rcljava_examples/java/rcljava_examples.jar:install_isolated/example_interfaces/share/example_interfaces/java/example_interfaces.jar:install_isolated/rcljava_common/share/rcljava_common/java/rcljava_common.jar org.ros2.rcljava.examples.AddTwoIntsService
```

You can also combine any scenario where the talker/listener or client/service are written in Java, Python and C++ and they should talk to each other.

Android
-------

The Android setup is slightly more complex, you'll need the SDK and NDK installed, and an Android device where you can run the examples.

Make sure to download at least the SDK for Android Lollipop (or greater), the examples require the API level 21 at least.

You may download the Android NDK from [the official](https://developer.android.com/ndk/downloads/index.html) website, let's assume you unpack it to `~/android_ndk`

```
mkdir -p ~/ros2_android_ws/src
cd ~/ros2_android_ws
wget https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_android.repos
vcs import ~/ros2_android_ws/src < ros2_java_android.repos
cd ~/ros2_android_ws/src/ros2/rosidl
patch -p1 < ../../ros2_java/ros2_java/rosidl_ros2_android.diff
touch python_cmake_module/AMENT_IGNORE
touch rosidl_generator_py/AMENT_IGNORE
cd ~/ros2_android_ws/src/eProsima/Fast-RTPS
git submodule init
git submodule update
cd ~/ros2_android_ws
. ~/ament_ws/install_isolated/local_setup.sh
ament build --isolated --cmake-args \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_TOOLCHAIN_FILE=$HOME/ros2_android_ws/src/android-cmake/android-cmake/android.toolchain.cmake \
  -DANDROID_FUNCTION_LEVEL_LINKING=OFF -DANDROID_NATIVE_API_LEVEL=android-21 \ -DANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang -DANDROID_STL=gnustl_shared \
  -DANDROID_ABI=armeabi-v7a -DANDROID_NDK=$HOME/android_ndk/android-ndk-r12b -DTHIRDPARTY=ON -DCOMPILE_EXAMPLES=OFF -- \
  --ament-cmake-args \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_TOOLCHAIN_FILE=$HOME/ros2_android_ws/src/android-cmake/android-cmake/android.toolchain.cmake \
  -DANDROID_FUNCTION_LEVEL_LINKING=OFF -DANDROID_NATIVE_API_LEVEL=android-21 \ -DANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang -DANDROID_STL=gnustl_shared \
  -DANDROID_ABI=armeabi-v7a -DANDROID_NDK=$HOME/android_ndk/android-ndk-r12b -DTHIRDPARTY=ON -DCOMPILE_EXAMPLES=OFF -- \
  --ament-gradle-args \
  -Pament.android_stl=gnustl_shared -Pament.android_abi=armeabi-v7a -Pament.android_ndk=$HOME/android_ndk/android-ndk-r12b --
```

The talker and listener example Android apps can be installed via adb, plug your Android device to your computer with a USB cable and type the following:

Talker:

```
adb install ~/ros2_android_ws/install_isolated/ros2_talker_android/ros2_talker_android-debug.apk
```

Listener:

```
adb install ~/ros2_android_ws/install_isolated/ros2_listener_android/ros2_listener_android-debug.apk
```

You can try out running the talker on the desktop and the listener on your Android device or viceversa.

Enjoy!

Acknowledgements
----------------

Thanks all those who have contributed:

Mickael Gaillard (https://github.com/Theosakamg)

TODO
----

There's a bunch of features missing, efficient intraprocess communication, DDS domain separation and QoS profiles.

Large messages would benefit from Java's NIO.

And of course, this wouldn't be a proper opensource project if it didn't lack tests and documentation, so there's that too.

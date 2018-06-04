// Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* DO NOT EDIT THIS FILE - it is machine generated */

/* Header for class org_ros2_rcljava_node_NativeNode */

#ifndef RCLJAVA__ORG_ROS2_RCLJAVA_EXECUTOR_NATIVEEXECUTOR_HPP_
#define RCLJAVA__ORG_ROS2_RCLJAVA_EXECUTOR_NATIVEEXECUTOR_HPP_

#include <jni.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeGetZeroInitializedWaitSet
 * Signature: ()J
 */
JNIEXPORT jlong JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeGetZeroInitializedWaitSet
  (JNIEnv *, jclass);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetInit
 * Signature: (JIIIII)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetInit
  (JNIEnv *, jclass, jlong, jint, jint, jint, jint, jint);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetClearSubscriptions
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearSubscriptions
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetAddSubscription
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddSubscription
  (JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWait
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWait
  (JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetFini
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetFini
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetClearServices
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearServices
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetAddService
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddService
  (JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetClearClients
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearClients
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetAddClient
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddClient
  (JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetClearTimers
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearTimers
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeWaitSetAddTimer
 * Signature: (JJ)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddTimer
  (JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeTake
 * Signature: (JLjava/lang/Class;)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeTake
  (JNIEnv *, jclass, jlong, jclass);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeTakeRequest
 * Signature: (JJJLjava/lang/Object;)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeTakeRequest
  (JNIEnv *, jclass, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeTakeResponse
 * Signature: (JJJLjava/lang/Object;)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeTakeResponse
  (JNIEnv *, jclass, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_executor_NativeExecutor
 * Method:    nativeSendServiceResponse
 * Signature: (JLjava/lang/Object;JJLjava/lang/Object;)V
 */
JNIEXPORT void JNICALL
  Java_org_ros2_rcljava_executor_NativeExecutor_nativeSendServiceResponse
  (JNIEnv *, jclass, jlong, jobject, jlong, jlong, jobject);

#ifdef __cplusplus
}
#endif
#endif  // RCLJAVA__ORG_ROS2_RCLJAVA_EXECUTOR_NATIVEEXECUTOR_HPP_
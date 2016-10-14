// Copyright 2016 Esteve Fernandez <esteve@apache.org>
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

#include <jni.h>
/* Header for class org_ros2_rcljava_Node */

#ifndef ORG_ROS2_RCLJAVA_NODE_H_
#define ORG_ROS2_RCLJAVA_NODE_H_

#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     org_ros2_rcljava_Node
 * Method:    nativeCreatePublisherHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreatePublisherHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

/*
 * Class:     org_ros2_rcljava_Node
 * Method:    nativeCreateSubscriptionHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreateSubscriptionHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

/*
 * Class:     org_ros2_rcljava_Node
 * Method:    nativeCreateServiceHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreateServiceHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

/*
 * Class:     org_ros2_rcljava_Node
 * Method:    nativeCreateClientHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreateClientHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

#ifdef __cplusplus
}
#endif

#endif  // ORG_ROS2_RCLJAVA_NODE_H_

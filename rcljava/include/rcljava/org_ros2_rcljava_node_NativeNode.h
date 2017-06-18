// Copyright 2016-2017 Esteve Fernandez <esteve@apache.org>
// Copyright 2016-2017 Mickael Gaillard <mick.gaillard@gmail.com>
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
#include <jni.h>
/* Header for class org_ros2_rcljava_node_NativeNode */

#ifndef RCLJAVA__ORG_ROS2_RCLJAVA_NODE_NATIVENODE_H_
#define RCLJAVA__ORG_ROS2_RCLJAVA_NODE_NATIVENODE_H_
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeCreatePublisherHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeCreatePublisherHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeCreateSubscriptionHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeCreateSubscriptionHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeCreateClientHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeCreateClientHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeCreateServiceHandle
 * Signature: (JLjava/lang/Class;Ljava/lang/String;J;)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeCreateServiceHandle
  (JNIEnv *, jclass, jlong, jclass, jstring, jlong);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeDispose
 * Signature: (J)V
 */
JNIEXPORT void JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeDispose
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeGetName
 * Signature: (J)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeGetName
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeCountPublishers
 * Signature: (JLjava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeCountPublishers
  (JNIEnv *, jclass, jlong, jstring);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeCountSubscribers
 * Signature: (JLjava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeCountSubscribers
  (JNIEnv *, jclass, jlong, jstring);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeGetListTopics
 * Signature: (JZ)Ljava/util/HashMap;
 */
JNIEXPORT jobject JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeGetListTopics
  (JNIEnv *, jclass, jlong, jboolean);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeGetListServices
 * Signature: (J)Ljava/util/HashMap;
 */
JNIEXPORT jobject JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeGetListServices
  (JNIEnv *, jclass, jlong);


/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeGetNodeNames
 * Signature: (J)Ljava/util/List;
 */
JNIEXPORT jobject JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeGetNodeNames
  (JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_node_NativeNode
 * Method:    nativeCreateTimerHandle
 * Signature: (J)J
 */
JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_node_NativeNode_nativeCreateTimerHandle
  (JNIEnv *, jclass, jlong);

#ifdef __cplusplus
}
#endif
#endif  // RCLJAVA__ORG_ROS2_RCLJAVA_NODE_NATIVENODE_H_

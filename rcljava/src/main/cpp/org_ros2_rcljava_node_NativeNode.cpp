// Copyright 2016-2017 Esteve Fernandez <esteve@apache.org>
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

#include <jni.h>

#include <string>
#include <cstdlib>
#include <cstdio>
#include <cassert>

#include "rmw/rmw.h"
#include "rcutils/types.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rcl/node.h"
#include "rcl/graph.h"
#include "rosidl_generator_c/message_type_support_struct.h"

#include "rcljava_common/exceptions.h"
#include "rcljava_common/signatures.h"

#include "rcljava/org_ros2_rcljava_node_NativeNode.h"
#include "rcljava/utils.hpp"

/*
 *
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeCreateNodeHandle(
  JNIEnv * env,
  jclass,
  jstring jnode_name,
  jstring jspace_name)
{
  std::string node_name = jstring2String(env, jnode_name);
  std::string space_name("");
  if (jspace_name != nullptr) {
    space_name = jstring2String(env, jspace_name);
  }

  rcl_node_t * node = makeInstance<rcl_node_t>();
  *node = rcl_get_zero_initialized_node();

  rcl_node_options_t default_options = rcl_node_get_default_options();

  rcl_ret_t ret = rcl_node_init(node, node_name.c_str(), space_name.c_str(), &default_options);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to create node: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);

    return 0;
  }

  jlong node_handle = instance2Handle(node);
  return node_handle;
}

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeDispose(
  JNIEnv * env,
  jclass,
  jlong jnode_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);

  rcl_ret_t ret = rcl_node_fini(node);
  if (ret != RCL_RET_OK) {
    std::string message("Failed finish node: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 *
 */
JNIEXPORT jstring JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeGetName(
  JNIEnv * env,
  jclass,
  jlong jnode_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);

  const char * name_tmp = rcl_node_get_name(node);
  jstring name = env->NewStringUTF(name_tmp);

  return name;
}

/*
 *
 */
JNIEXPORT jint JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeCountPublishers(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jstring jtopic)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  std::string topic = jstring2String(env, jtopic);

  size_t count = -1;
  rcl_ret_t ret = rcl_count_publishers(node, topic.c_str(), &count);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to count Publishers: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  return count;
}

/*
 *
 */
JNIEXPORT jint JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeCountSubscribers(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jstring jtopic)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  std::string topic = jstring2String(env, jtopic);

  size_t count = 0;
  rcl_ret_t ret = rcl_count_subscribers(node, topic.c_str(), &count);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to count Publishers: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  return count;
}

JNIEXPORT jobject JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeGetListTopics(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jboolean no_demangle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rcl_names_and_types_t topic_names_and_types =
    rcl_get_zero_initialized_names_and_types();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_topic_names_and_types(node,
      &allocator,
      no_demangle,
      &topic_names_and_types);

  if (ret != RCL_RET_OK) {
    std::string message("Failed get list of topics: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  jobject topics = makeJTopics(env, &topic_names_and_types);

  ret = rcl_names_and_types_fini(&topic_names_and_types);
  if (ret != RCL_RET_OK) {
    std::string message("Failed get list of topics: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  return topics;
}

JNIEXPORT jobject JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeGetListServices(
  JNIEnv * env,
  jclass,
  jlong jnode_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rcl_names_and_types_t service_names_and_types =
    rcl_get_zero_initialized_names_and_types();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_service_names_and_types(node,
      &allocator,
      &service_names_and_types);

  if (ret != RCL_RET_OK) {
    std::string message("Failed get list of services: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  jobject services = makeJTopics(env, &service_names_and_types);

  ret = rcl_names_and_types_fini(&service_names_and_types);
  if (ret != RCL_RET_OK) {
    std::string message("Failed get list of service: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  return services;
}

JNIEXPORT jobject JNICALL
Java_org_ros2_rcljava_node_NativeNode_nativeGetNodeNames(
  JNIEnv * env,
  jclass,
  jlong jnode_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rcutils_string_array_t node_names = rcutils_get_zero_initialized_string_array();

  auto allocator = rcl_get_default_allocator();
  rcl_ret_t ret = rcl_get_node_names(node, allocator, &node_names);
  if (ret != RCL_RET_OK) {
    std::string message("Failed get list of nodes: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  jobject nodes = makeJNodes(env, &node_names);

  ret = rcutils_string_array_fini(&node_names);
  if (ret != RCL_RET_OK) {
    std::string message("Failed get list of nodes: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  return nodes;
}

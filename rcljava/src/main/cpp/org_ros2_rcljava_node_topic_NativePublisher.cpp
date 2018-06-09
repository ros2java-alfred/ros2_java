// Copyright 2016 Esteve Fernandez <esteve@apache.org>
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
#include <rcljava/org_ros2_rcljava_node_topic_NativePublisher.hpp>
#include <rcljava/utils.hpp>

#include <string>
#include <cstdlib>
#include <cassert>
#include <cstdio>

#include "rmw/rmw.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rcl/node.h"

#include "rcljava_common/exceptions.h"
#include "rcljava_common/signatures.h"

/*
 * nativeCreatePublisherHandle
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_node_topic_NativePublisher_nativeCreatePublisherHandle(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jclass jmessage_class,
  jstring jtopic,
  jlong qos_profile_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rosidl_message_type_support_t * msg_type = jclass2MessageType(env, jmessage_class);
  std::string topic = jstring2String(env, jtopic);

  rcl_publisher_t * publisher = makeInstance<rcl_publisher_t>();
  *publisher = rcl_get_zero_initialized_publisher();

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  rmw_qos_profile_t * qos_profile = handle2Instance<rmw_qos_profile_t>(qos_profile_handle);
  publisher_ops.qos = *qos_profile;

  rcl_ret_t ret = rcl_publisher_init(
    publisher,
    node,
    msg_type,
    topic.c_str(),
    &publisher_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create publisher: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);

    return -1;
  }

  jlong publisher_handle = instance2Handle(publisher);
  return publisher_handle;
}

/*
 * nativePublish
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_node_topic_NativePublisher_nativePublish(
  JNIEnv * env,
  jclass,
  jlong jpublisher_handle,
  jobject jmsg)
{
  rcl_publisher_t * publisher = handle2Instance<rcl_publisher_t>(jpublisher_handle);

  void * raw_ros_message = jobject2Message(env, jmsg);
  if (env->ExceptionCheck()) {
    return;
  }

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to publish: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 * nativeDispose
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_node_topic_NativePublisher_nativeDispose(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jlong jpublisher_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rcl_publisher_t * publisher = handle2Instance<rcl_publisher_t>(jpublisher_handle);

  rcl_ret_t ret = rcl_publisher_fini(publisher, node);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to destroy publisher: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

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
#include <rcljava/org_ros2_rcljava_node_topic_NativeSubscription.hpp>
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
 * nativeCreateSubscriptionHandle
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_node_topic_NativeSubscription_nativeCreateSubscriptionHandle(
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

  rcl_subscription_t * subscription = makeInstance<rcl_subscription_t>();
  *subscription = rcl_get_zero_initialized_subscription();

  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();
  rmw_qos_profile_t * qos_profile = handle2Instance<rmw_qos_profile_t>(qos_profile_handle);
  subscription_ops.qos = *qos_profile;

  rcl_ret_t ret = rcl_subscription_init(
    subscription,
    node,
    msg_type,
    topic.c_str(),
    &subscription_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create subscription: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);

    return -1;
  }

  jlong jsubscription = instance2Handle(subscription);
  return jsubscription;
}

/*
 * nativeDispose
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_node_topic_NativeSubscription_nativeDispose(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jlong jsubscription_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rcl_subscription_t * subscription = handle2Instance<rcl_subscription_t>(jsubscription_handle);

  rcl_ret_t ret = rcl_subscription_fini(subscription, node);
  free(subscription);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to destroy subscription: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

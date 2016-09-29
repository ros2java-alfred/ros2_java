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
#include <string>
#include <cstdlib>
#include <cassert>
#include <cstdio>

#include "rmw/rmw.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rcl/node.h"
#include "rosidl_generator_c/message_type_support_struct.h"

#include "rcljava_common/exceptions.h"
#include "rcljava_common/signatures.h"

#include "org_ros2_rcljava_Node.h"

JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreatePublisherHandle(JNIEnv * env, jclass,
  jlong node_handle,
  jclass jmessage_class,
  jstring jtopic)
{
  jmethodID mid = env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");
  jlong jts = env->CallStaticLongMethod(jmessage_class, mid);

  const char * topic_tmp = env->GetStringUTFChars(jtopic, 0);

  std::string topic(topic_tmp);

  env->ReleaseStringUTFChars(jtopic, topic_tmp);

  rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);

  rosidl_message_type_support_t * ts =
    reinterpret_cast<rosidl_message_type_support_t *>(jts);

  rcl_publisher_t * publisher = static_cast<rcl_publisher_t *>(malloc(sizeof(rcl_publisher_t)));
  publisher->impl = NULL;
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic.c_str(), &publisher_ops);

  if (ret != RCL_RET_OK) {
    rcljava_throw_exception(
      env, "java/lang/IllegalStateException",
      "Failed to create publisher: " + std::string(rcl_get_error_string_safe()));
    return 0;
  }

  jlong jpublisher = reinterpret_cast<jlong>(publisher);
  return jpublisher;
}

JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreateSubscriptionHandle(JNIEnv * env,
  jclass,
  jlong node_handle,
  jclass jmessage_class,
  jstring jtopic)
{
  jmethodID mid = env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");
  jlong jts = env->CallStaticLongMethod(jmessage_class, mid);

  const char * topic_tmp = env->GetStringUTFChars(jtopic, 0);

  std::string topic(topic_tmp);

  env->ReleaseStringUTFChars(jtopic, topic_tmp);

  rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);

  rosidl_message_type_support_t * ts =
    reinterpret_cast<rosidl_message_type_support_t *>(jts);

  rcl_subscription_t * subscription =
    static_cast<rcl_subscription_t *>(malloc(sizeof(rcl_subscription_t)));
  subscription->impl = NULL;
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic.c_str(), &subscription_ops);

  if (ret != RCL_RET_OK) {
    rcljava_throw_exception(
      env, "java/lang/IllegalStateException",
      "Failed to create subscription: " + std::string(rcl_get_error_string_safe()));
    return 0;
  }

  jlong jsubscription = reinterpret_cast<jlong>(subscription);
  return jsubscription;
}

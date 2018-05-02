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
#include <rcljava/utils.hpp>

#include <cassert>
#include <cstdlib>
#include <string>

#include "rmw/rmw.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rcl/node.h"
#include "rosidl_generator_c/message_type_support_struct.h"

#include "rcljava_common/exceptions.h"
#include "rcljava_common/signatures.h"

#include "rcljava/org_ros2_rcljava_node_service_NativeClient.h"


/*
 * nativeCreateClientHandle
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_node_service_NativeClient_nativeCreateClientHandle(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jclass jservice_class,
  jstring jservice_topic,
  jlong qos_profile_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rosidl_service_type_support_t * msg_type = jclass2ServiceType(env, jservice_class);
  std::string service_topic = jstring2String(env, jservice_topic);

  rcl_client_t * client = makeInstance<rcl_client_t>();
  *client = rcl_get_zero_initialized_client();

//  bool is_available = false;
//  rcl_ret_t ret = rcl_service_server_is_available(node, client, &is_available);
//  printf("===> is available\n");
//  if (ret != RCL_RET_OK || !is_available) {
//    std::string message("Failed to connect to server: " +
//        std::string(rcl_get_error_string_safe()));
//        rcl_reset_error();
//    throwException(env, message);
//
//    return -1;
//  }

  rcl_client_options_t client_ops = rcl_client_get_default_options();
  rmw_qos_profile_t * qos_profile = reinterpret_cast<rmw_qos_profile_t *>(qos_profile_handle);
  client_ops.qos = *qos_profile;

  rcl_ret_t ret = rcl_client_init(
    client,
    node,
    msg_type,
    service_topic.c_str(),
    &client_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create client: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);

    return -1;
  }

  jlong jclient = instance2Handle(client);
  return jclient;
}

/*
 * nativeSendClientRequest
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_node_service_NativeClient_nativeSendClientRequest(
  JNIEnv * env,
  jclass,
  jlong client_handle,
  jlong sequence_number,
  jlong jrequest_from_java_converter_handle,
  jlong jrequest_to_java_converter_handle,
  jobject jrequest_msg)
{
  assert(client_handle != 0);
  assert(jrequest_from_java_converter_handle != 0);
  assert(jrequest_to_java_converter_handle != 0);
  assert(jrequest_msg != nullptr);

  rcl_client_t * client = handle2Instance<rcl_client_t>(client_handle);

  convert_from_java_signature convert_from_java =
    reinterpret_cast<convert_from_java_signature>(jrequest_from_java_converter_handle);

  void * request_msg = convert_from_java(jrequest_msg, nullptr);

  rcl_ret_t ret = rcl_send_request(client, request_msg, &sequence_number);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to send request from a client: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 * nativeDispose
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_node_service_NativeClient_nativeDispose(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jlong jclient_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rcl_client_t * client = handle2Instance<rcl_client_t>(jclient_handle);

  rcl_ret_t ret = rcl_client_fini(client, node);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to destroy client: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

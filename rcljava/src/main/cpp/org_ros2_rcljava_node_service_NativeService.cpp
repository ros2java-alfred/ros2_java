// Copyright 2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

#include "rcljava/org_ros2_rcljava_node_service_NativeService.h"


/*
 * nativeCreateServiceHandle
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_node_service_NativeService_nativeCreateServiceHandle(
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

  rcl_service_t * service = makeInstance<rcl_service_t>();
  *service = rcl_get_zero_initialized_service();

  rcl_service_options_t service_ops = rcl_service_get_default_options();
  rmw_qos_profile_t * qos_profile = handle2Instance<rmw_qos_profile_t>(qos_profile_handle);
  service_ops.qos = *qos_profile;

  rcl_ret_t ret = rcl_service_init(
    service,
    node,
    msg_type,
    service_topic.c_str(),
    &service_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create service: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);

    return -1;
  }

  jlong jservice = instance2Handle(service);
  return jservice;
}

/*
 * nativeDispose
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_node_service_NativeService_nativeDispose(
  JNIEnv * env,
  jclass,
  jlong jnode_handle,
  jlong jservice_handle)
{
  rcl_node_t * node = handle2Instance<rcl_node_t>(jnode_handle);
  rcl_service_t * service = handle2Instance<rcl_service_t>(jservice_handle);

  rcl_ret_t ret = rcl_service_fini(service, node);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to destroy service: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

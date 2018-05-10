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
#include <rcljava/org_ros2_rcljava_RCLJava.hpp>
#include <rcljava/utils.hpp>

#include <cassert>
#include <cstdlib>
#include <string>

#include "rmw/rmw.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rcl/node.h"
#include "rcl/timer.h"

#include "rosidl_generator_c/message_type_support_struct.h"

#include "rcljava_common/exceptions.h"
#include "rcljava_common/signatures.h"

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_RCLJava_nativeRCLJavaInit(
  JNIEnv * env,
  jclass,
  jobjectArray arg)
{
  int argc = 0;
  char ** argv = nullptr;

  if (arg != NULL) {
    argc = env->GetArrayLength(arg);
    argv = JniStringArray2StringArray(env, arg);
  }

  rcl_ret_t ret = rcl_init(argc, argv, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    std::string message("failed to initialize rmw implementation: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_RCLJava_nativeShutdown(
  JNIEnv * env,
  jclass)
{
  rcl_ret_t ret = rcl_shutdown();
  if (ret != RCL_RET_OK) {
    std::string message("Failed to shutdown: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 *
 */
JNIEXPORT jboolean JNICALL
Java_org_ros2_rcljava_RCLJava_nativeOk(
  JNIEnv *,
  jclass)
{
  return rcl_ok();
}

/*
 *
 */
JNIEXPORT jstring JNICALL
Java_org_ros2_rcljava_RCLJava_nativeGetRMWIdentifier(
  JNIEnv * env,
  jclass)
{
  const char * rmw_implementation_identifier = rmw_get_implementation_identifier();

  return env->NewStringUTF(rmw_implementation_identifier);
}

JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_RCLJava_nativeConvertQoSProfileToHandle(
  JNIEnv *,
  jclass,
  jint history,
  jint depth,
  jint reliability,
  jint durability,
  jboolean avoid_ros_namespace_conventions)
{
  rmw_qos_profile_t * qos_profile = makeInstance<rmw_qos_profile_t>();

  qos_profile->history = static_cast<rmw_qos_history_policy_t>(history);
  qos_profile->depth = depth;
  qos_profile->reliability = static_cast<rmw_qos_reliability_policy_t>(reliability);
  qos_profile->durability = static_cast<rmw_qos_durability_policy_t>(durability);
  qos_profile->avoid_ros_namespace_conventions = avoid_ros_namespace_conventions;

  return instance2Handle(qos_profile);
}

JNIEXPORT void JNICALL
Java_org_ros2_rcljava_RCLJava_nativeDisposeQoSProfile(
  JNIEnv *,
  jclass,
  jlong qos_profile_handle)
{
  rmw_qos_profile_t * qos_profile =
    handle2Instance<rmw_qos_profile_t>(qos_profile_handle);

  free(qos_profile);
}

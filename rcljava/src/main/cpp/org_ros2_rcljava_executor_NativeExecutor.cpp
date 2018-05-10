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
#include <rcljava/org_ros2_rcljava_executor_NativeExecutor.hpp>
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

jobject convert_rmw_request_id_to_java(JNIEnv *, rmw_request_id_t *);

rmw_request_id_t * convert_rmw_request_id_from_java(JNIEnv *, jobject);

/*
 *
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeGetZeroInitializedWaitSet(
  JNIEnv *,
  jclass)
{
  rcl_wait_set_t * wait_set = makeInstance<rcl_wait_set_t>();
  *wait_set = rcl_get_zero_initialized_wait_set();
//  wait_set->subscriptions = nullptr;
//  wait_set->size_of_subscriptions = 0;
//  wait_set->guard_conditions = nullptr;
//  wait_set->size_of_guard_conditions = 0;
//  wait_set->timers = nullptr;
//  wait_set->size_of_timers = 0;
//  wait_set->clients = nullptr;
//  wait_set->size_of_clients = 0;
//  wait_set->services = nullptr;
//  wait_set->size_of_services = 0;
//  wait_set->impl = nullptr;

  jlong wait_set_handle = instance2Handle(wait_set);
  return wait_set_handle;
}

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetInit(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle,
  jint number_of_subscriptions,
  jint number_of_guard_conditions,
  jint number_of_timers,
  jint number_of_clients,
  jint number_of_services
)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);

  rcl_ret_t ret = rcl_wait_set_init(
    wait_set,
    number_of_subscriptions,
    number_of_guard_conditions,
    number_of_timers,
    number_of_clients,
    number_of_services,
    rcl_get_default_allocator());

  if (ret != RCL_RET_OK) {
    std::string message("Failed to initialize wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearSubscriptions(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);

  rcl_ret_t ret = rcl_wait_set_clear_subscriptions(wait_set);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to clear subscriptions from wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddSubscription(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle,
  jlong subscription_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);
  rcl_subscription_t * subscription =
    handle2Instance<rcl_subscription_t>(subscription_handle);

  rcl_ret_t ret = rcl_wait_set_add_subscription(wait_set, subscription);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to add subscription to wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWait(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle,
  jlong timeout)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);

  rcl_ret_t ret = rcl_wait(wait_set, RCL_MS_TO_NS(timeout));
  if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
    std::string message("Failed to wait on wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/*
 *
 */
JNIEXPORT jobject JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeTake(
  JNIEnv * env,
  jclass,
  jlong subscription_handle,
  jclass jmessage_class)
{
  rcl_subscription_t * subscription =
    handle2Instance<rcl_subscription_t>(subscription_handle);

  void * taken_msg = jclass2Message(env, jmessage_class);
  if (env->ExceptionCheck()) {
    return 0;
  }

  rcl_ret_t ret = rcl_take(subscription, taken_msg, nullptr);
  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    std::string message("Failed to take from a subscription: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
    return 0;
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    jobject jtaken_msg = jclass2JMessage(env, jmessage_class, taken_msg);

    return jtaken_msg;
  }

  return nullptr;
}

/*
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetFini(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);

  rcl_ret_t ret = rcl_wait_set_fini(wait_set);
  free(wait_set);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to release wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearServices(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);

  rcl_ret_t ret = rcl_wait_set_clear_services(wait_set);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to clear services from wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddService(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle,
  jlong service_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);
  rcl_service_t * service = handle2Instance<rcl_service_t>(service_handle);

  rcl_ret_t ret = rcl_wait_set_add_service(wait_set, service);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to add service to wait set: " +
      std::string(rcl_get_error_string_safe()));
    throwException(env, message);
  }
}

JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearClients(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);

  rcl_ret_t ret = rcl_wait_set_clear_clients(wait_set);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to clear clients from wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddClient(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle,
  jlong client_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);
  rcl_client_t * client = handle2Instance<rcl_client_t>(client_handle);

  rcl_ret_t ret = rcl_wait_set_add_client(wait_set, client);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to add client to wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}


JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetClearTimers(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);

  rcl_ret_t ret = rcl_wait_set_clear_timers(wait_set);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to clear timers from wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeWaitSetAddTimer(
  JNIEnv * env,
  jclass,
  jlong wait_set_handle,
  jlong timer_handle)
{
  rcl_wait_set_t * wait_set = handle2Instance<rcl_wait_set_t>(wait_set_handle);
  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(timer_handle);

  rcl_ret_t ret = rcl_wait_set_add_timer(wait_set, timer);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to add timer to wait set: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

JNIEXPORT jobject JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeTakeRequest(
  JNIEnv * env,
  jclass,
  jlong service_handle,
  jlong jrequest_from_java_converter_handle,
  jlong jrequest_to_java_converter_handle,
  jobject jrequest_msg)
{
  assert(service_handle != 0);
  assert(jrequest_from_java_converter_handle != 0);
  assert(jrequest_to_java_converter_handle != 0);
  assert(jrequest_msg != nullptr);

  rcl_service_t * service = handle2Instance<rcl_service_t>(service_handle);

  convert_from_java_signature convert_from_java =
    reinterpret_cast<convert_from_java_signature>(jrequest_from_java_converter_handle);

  convert_to_java_signature convert_to_java =
    reinterpret_cast<convert_to_java_signature>(jrequest_to_java_converter_handle);

  void * taken_msg = convert_from_java(jrequest_msg, nullptr);

  rmw_request_id_t header;

  rcl_ret_t ret = rcl_take_request(service, &header, taken_msg);
  if (ret != RCL_RET_OK && ret != RCL_RET_SERVICE_TAKE_FAILED) {
    std::string message("Failed to take request from a service: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
    return nullptr;
  }

  if (ret != RCL_RET_SERVICE_TAKE_FAILED) {
    jobject jtaken_msg = convert_to_java(taken_msg, jrequest_msg);
    assert(jtaken_msg != nullptr);

    jobject jheader = convert_rmw_request_id_to_java(env, &header);
    return jheader;
  }

  return nullptr;
}

JNIEXPORT void JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeSendServiceResponse(
  JNIEnv * env,
  jclass,
  jlong service_handle,
  jobject jrequest_id,
  jlong jresponse_from_java_converter_handle,
  jlong jresponse_to_java_converter_handle,
  jobject jresponse_msg)
{
  assert(service_handle != 0);
  assert(jresponse_from_java_converter_handle != 0);
  assert(jresponse_to_java_converter_handle != 0);
  assert(jresponse_msg != nullptr);

  rcl_service_t * service = handle2Instance<rcl_service_t>(service_handle);

  convert_from_java_signature convert_from_java =
    reinterpret_cast<convert_from_java_signature>(jresponse_from_java_converter_handle);

  void * response_msg = convert_from_java(jresponse_msg, nullptr);

  rmw_request_id_t * request_id = convert_rmw_request_id_from_java(env, jrequest_id);

  rcl_ret_t ret = rcl_send_response(service, request_id, response_msg);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to send response from a service: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

JNIEXPORT jobject JNICALL
Java_org_ros2_rcljava_executor_NativeExecutor_nativeTakeResponse(
  JNIEnv * env,
  jclass,
  jlong client_handle,
  jlong jresponse_from_java_converter_handle,
  jlong jresponse_to_java_converter_handle,
  jobject jresponse_msg)
{
  assert(client_handle != 0);
  assert(jresponse_from_java_converter_handle != 0);
  assert(jresponse_to_java_converter_handle != 0);
  assert(jresponse_msg != nullptr);

  rcl_client_t * client = handle2Instance<rcl_client_t>(client_handle);

  convert_from_java_signature convert_from_java =
    reinterpret_cast<convert_from_java_signature>(jresponse_from_java_converter_handle);

  convert_to_java_signature convert_to_java =
    reinterpret_cast<convert_to_java_signature>(jresponse_to_java_converter_handle);

  void * taken_msg = convert_from_java(jresponse_msg, nullptr);

  rmw_request_id_t header;

  rcl_ret_t ret = rcl_take_response(client, &header, taken_msg);
  if (ret != RCL_RET_OK && ret != RCL_RET_CLIENT_TAKE_FAILED) {
    std::string message("Failed to take request from a service: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
    return nullptr;
  }

  if (ret != RCL_RET_CLIENT_TAKE_FAILED) {
    jobject jtaken_msg = convert_to_java(taken_msg, jresponse_msg);

    assert(jtaken_msg != nullptr);

    jobject jheader = convert_rmw_request_id_to_java(env, &header);
    return jheader;
  }

  return nullptr;
}

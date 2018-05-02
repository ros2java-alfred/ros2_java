// Copyright 2017 Esteve Fernandez <esteve@apache.org>
// Copyright 2017-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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
#include <cstdio>
#include <cstdlib>
#include <string>

#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rcl/rcl.h"
#include "rmw/rmw.h"

#include "rcljava_common/exceptions.h"
#include "rcljava_common/signatures.h"

#include "rcljava/org_ros2_rcljava_time_NativeWallTimer.h"


JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeCreateTimerHandle(
  JNIEnv * env,
  jclass,
  jlong timer_period)
{
  rcl_timer_t * timer = makeInstance<rcl_timer_t>();
  *timer = rcl_get_zero_initialized_timer();

  rcl_ret_t ret = rcl_timer_init(timer, timer_period, NULL, rcl_get_default_allocator());

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create timer: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
    return 0;
  }

  jlong jtimer = instance2Handle(timer);
  return jtimer;
}

/**
 *
 */
JNIEXPORT jboolean JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeIsReady(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  bool is_ready;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &is_ready);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to check timer ready: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  return is_ready;
}

/**
 *
 */
JNIEXPORT jboolean JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeIsCanceled(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  bool is_canceled;
  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to check timer canceled: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }

  return is_canceled;
}

/**
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeDispose(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  if (wall_timer_handle == 0) {
    // everything is ok, already destroyed
    return;
  }

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  rcl_ret_t ret = rcl_timer_fini(timer);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to destroy timer: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/**
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeReset(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  rcl_ret_t ret = rcl_timer_reset(timer);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to reset timer: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/**
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeCancel(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  rcl_ret_t ret = rcl_timer_cancel(timer);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to cancel timer: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/**
 *
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeTimeUntilNextCall(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  int64_t remaining_time;
  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &remaining_time);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to get time until next timer call: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
    return 0;
  }

  return remaining_time;
}

/**
 *
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeTimeSinceLastCall(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  int64_t elapsed_time;
  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &elapsed_time);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to get time until next timer call: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
    return 0;
  }

  return elapsed_time;
}

/**
 *
 */
JNIEXPORT jlong JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeGetTimerPeriodNS(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  int64_t timer_period;
  rcl_ret_t ret = rcl_timer_get_period(timer, &timer_period);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to get timer period: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
    return 0;
  }

  return timer_period;
}

/**
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeSetTimerPeriodNS(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle,
  jlong timer_period)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  int64_t old_period;
  rcl_ret_t ret = rcl_timer_exchange_period(timer, timer_period, &old_period);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to set timer period: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

/**
 *
 */
JNIEXPORT void JNICALL
Java_org_ros2_rcljava_time_NativeWallTimer_nativeCallTimer(
  JNIEnv * env,
  jclass,
  jlong wall_timer_handle)
{
  assert(wall_timer_handle != 0);

  rcl_timer_t * timer = handle2Instance<rcl_timer_t>(wall_timer_handle);

  assert(timer != NULL);

  rcl_ret_t ret = rcl_timer_call(timer);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to call timer: " +
      std::string(rcl_get_error_string_safe()));
    rcl_reset_error();
    throwException(env, message);
  }
}

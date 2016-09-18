#include <string>
#include <cstdlib>
#include <cassert>
#include <cstdio>
#include <jni.h>

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>
#include <jni.h>

#include "org_ros2_rcljava_Publisher.h"
#include "utils.h"

/*
 * nativePublish
 */
JNIEXPORT void
JNICALL Java_org_ros2_rcljava_Publisher_nativePublish(
    JNIEnv *env,
    jclass ,
     jlong jpublisher_handle,
   jobject jmsg) {

  rcl_publisher_t *publisher = handle2Instance<rcl_publisher_t>(jpublisher_handle);

  void *raw_ros_message = jobject2Message(env, jmsg);

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to publish: " +
            std::string(rcl_get_error_string_safe()));
    throwException(env, message);
  }
}

/*
 * nativeDispose
 */
JNIEXPORT void
JNICALL Java_org_ros2_rcljava_Publisher_nativeDispose(
    JNIEnv *env,
    jclass ,
     jlong jnode_handle,
     jlong jpublisher_handle) {

  rcl_node_t *node = handle2Instance<rcl_node_t>(jnode_handle);
  rcl_publisher_t *publisher = handle2Instance<rcl_publisher_t>(jpublisher_handle);

  rcl_ret_t ret = rcl_publisher_fini(publisher, node);
  if (ret != RCL_RET_OK) {
    std::string message("Failed to destroy publisher: " +
        std::string(rcl_get_error_string_safe()));
    throwException(env, message);
  }
}

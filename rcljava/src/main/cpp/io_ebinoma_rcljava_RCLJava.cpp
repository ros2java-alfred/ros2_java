#include <jni.h>
#include <string>
#include <cassert>
#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>
#include <rosidl_generator_c/message_type_support.h>
#include <cstdlib>

#include "io_ebinoma_rcljava_RCLJava.h"

JNIEXPORT void JNICALL Java_io_ebinoma_rcljava_RCLJava_rcljavaInit
  (JNIEnv *env, jclass) {
  // TODO(esteve): parse args
  rcl_ret_t ret = rcl_init(0, NULL, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    jclass exception_class;
    const char *class_name = "java/lang/IllegalStateException";
    std::string message("Failed to init: " + std::string(rcl_get_error_string_safe()));

    exception_class = env->FindClass(class_name);

    assert(exception_class != NULL);

    env->ThrowNew(exception_class, message.c_str());
  }
}

JNIEXPORT jlong JNICALL Java_io_ebinoma_rcljava_RCLJava_createNodeHandle
  (JNIEnv *env, jclass, jstring jnode_name) {

  const char * node_name_tmp = env->GetStringUTFChars(jnode_name, 0);

  std::string node_name(node_name_tmp);

  env->ReleaseStringUTFChars(jnode_name, node_name_tmp);

  rcl_node_t * node = (rcl_node_t *)malloc(sizeof(rcl_node_t));
  node->impl = NULL;
  rcl_node_options_t default_options = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_init(node, node_name.c_str(), &default_options);
  if (ret != RCL_RET_OK) {
    jclass exception_class;
    const char *class_name = "java/lang/IllegalStateException";
    std::string message("Failed to create node: " + std::string(rcl_get_error_string_safe()));

    exception_class = env->FindClass(class_name);

    assert(exception_class != NULL);

    env->ThrowNew(exception_class, message.c_str());

    return -1;
  }
  jlong node_handle = reinterpret_cast<jlong>(node);
  return node_handle;
}

JNIEXPORT jstring JNICALL Java_io_ebinoma_rcljava_RCLJava_getTypesupportIdentifier
  (JNIEnv *env, jclass) {
  return env->NewStringUTF("foo");
}

JNIEXPORT jstring JNICALL Java_io_ebinoma_rcljava_RCLJava_getRMWIdentifier
  (JNIEnv *env, jclass) {
  return env->NewStringUTF("bar");
}

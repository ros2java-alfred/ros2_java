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

#include "io_ebinoma_rcljava_Publisher.h"

JNIEXPORT void JNICALL Java_io_ebinoma_rcljava_Publisher_nativePublish
  (JNIEnv *env, jclass, jlong publisher_handle, jobject jmsg) {

  rcl_publisher_t * publisher = reinterpret_cast<rcl_publisher_t *>(publisher_handle);

  jclass jmessage_class = env->GetObjectClass(jmsg);

  jmethodID mid = env->GetStaticMethodID(jmessage_class, "getFromJavaConverter", "()J");
  jlong jfrom_java_converter = env->CallStaticLongMethod(jmessage_class, mid);

  typedef void * (* convert_from_java_signature)(jobject);
  convert_from_java_signature convert_from_java =
    reinterpret_cast<convert_from_java_signature>(jfrom_java_converter);

  void * raw_ros_message = convert_from_java(jmsg);

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message);
  if (ret != RCL_RET_OK) {
    jclass exception_class;
    const char *class_name = "java/lang/IllegalStateException";
    std::string message("Failed to publish: " + std::string(rcl_get_error_string_safe()));

    exception_class = env->FindClass(class_name);

    assert(exception_class != NULL);

    env->ThrowNew(exception_class, message.c_str());
  }
}

#include <string>
#include <cstdlib>
#include <cassert>
#include <cstdio>
#include <jni.h>

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#include "org_ros2_rcljava_Node.h"

JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreatePublisherHandle
  (JNIEnv *env, jclass, jlong node_handle, jclass jmessage_class, jstring jtopic) {
  jmethodID mid = env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");
  jlong jts = env->CallStaticLongMethod(jmessage_class, mid);

  const char *topic_tmp = env->GetStringUTFChars(jtopic, 0);

  std::string topic(topic_tmp);

  env->ReleaseStringUTFChars(jtopic, topic_tmp);

  rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);

  rosidl_message_type_support_t * ts =
    reinterpret_cast<rosidl_message_type_support_t *>(jts);

  rcl_publisher_t * publisher = (rcl_publisher_t *)malloc(sizeof(rcl_publisher_t));
  publisher->impl = NULL;
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  rcl_ret_t ret = rcl_publisher_init(publisher, node, ts, topic.c_str(), &publisher_ops);

  if (ret != RCL_RET_OK) {
    jclass exception_class;
    const char *class_name = "java/lang/IllegalStateException";
    std::string message("Failed to create publisher: " + std::string(rcl_get_error_string_safe()));

    exception_class = env->FindClass(class_name);

    assert(exception_class != NULL);

    env->ThrowNew(exception_class, message.c_str());

    return -1;
  }

  jlong jpublisher = reinterpret_cast<jlong>(publisher);

  return jpublisher;
}

JNIEXPORT jlong JNICALL Java_org_ros2_rcljava_Node_nativeCreateSubscriptionHandle
  (JNIEnv *env, jclass, jlong node_handle, jclass jmessage_class, jstring jtopic) {
  jmethodID mid = env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");
  jlong jts = env->CallStaticLongMethod(jmessage_class, mid);

  const char *topic_tmp = env->GetStringUTFChars(jtopic, 0);

  std::string topic(topic_tmp);

  env->ReleaseStringUTFChars(jtopic, topic_tmp);

  rcl_node_t * node = reinterpret_cast<rcl_node_t *>(node_handle);

  rosidl_message_type_support_t * ts =
    reinterpret_cast<rosidl_message_type_support_t *>(jts);

  rcl_subscription_t * subscription = (rcl_subscription_t *)malloc(sizeof(rcl_subscription_t));
  subscription->impl = NULL;
  rcl_subscription_options_t subscription_ops = rcl_subscription_get_default_options();

  rcl_ret_t ret = rcl_subscription_init(subscription, node, ts, topic.c_str(), &subscription_ops);

  if (ret != RCL_RET_OK) {
    jclass exception_class;
    const char *class_name = "java/lang/IllegalStateException";
    std::string message("Failed to create publisher: " + std::string(rcl_get_error_string_safe()));

    exception_class = env->FindClass(class_name);

    assert(exception_class != NULL);

    env->ThrowNew(exception_class, message.c_str());

    return -1;
  }

  jlong jsubscription = reinterpret_cast<jlong>(subscription);

  return jsubscription;
}

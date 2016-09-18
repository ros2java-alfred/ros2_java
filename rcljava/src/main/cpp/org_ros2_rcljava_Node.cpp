#include <string>
#include <cstdlib>
#include <cstdio>
#include <jni.h>

#include <rmw/rmw.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/node.h>
#include <rosidl_generator_c/message_type_support_struct.h>

#include "org_ros2_rcljava_Node.h"
#include "utils.h"

/*
 * nativeCreatePublisherHandle
 */
JNIEXPORT jlong
JNICALL Java_org_ros2_rcljava_Node_nativeCreatePublisherHandle(
    JNIEnv *env,
    jclass ,
     jlong jnode_handle,
    jclass jmessage_class,
   jstring jtopic,
   jobject jqos) {

  rcl_node_t *node = handle2Instance<rcl_node_t>(jnode_handle);
  rosidl_message_type_support_t *msg_type = jclass2MessageType(env, jmessage_class);
  std::string topic = jstring2String(env, jtopic);

  rcl_publisher_t * publisher = makeInstance<rcl_publisher_t>();
  publisher->impl = NULL;

  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();
//  publisher_ops.qos =

  rcl_ret_t ret = rcl_publisher_init(
      publisher,
      node,
      msg_type,
      topic.c_str(),
      &publisher_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create publisher: " +
        std::string(rcl_get_error_string_safe()));
    throwException(env, message);

    return -1;
  }

  jlong publisher_handle = instance2Handle(publisher);
  return publisher_handle;
}

/*
 * nativeCreateSubscriptionHandle
 */
JNIEXPORT jlong
JNICALL Java_org_ros2_rcljava_Node_nativeCreateSubscriptionHandle(
    JNIEnv *env,
    jclass ,
     jlong jnode_handle,
    jclass jmessage_class,
   jstring jtopic, jobject jqos) {

  rcl_node_t *node = handle2Instance<rcl_node_t>(jnode_handle);
  rosidl_message_type_support_t *msg_type = jclass2MessageType(env, jmessage_class);
  std::string topic = jstring2String(env, jtopic);

  rcl_subscription_t * subscription = makeInstance<rcl_subscription_t>();
  subscription->impl = NULL;

  rcl_subscription_options_t subscription_ops =
      rcl_subscription_get_default_options();

  rcl_ret_t ret = rcl_subscription_init(
      subscription,
      node,
      msg_type,
      topic.c_str(),
      &subscription_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create subscription: " +
        std::string(rcl_get_error_string_safe()));
    throwException(env, message);

    return -1;
  }

  jlong jsubscription = instance2Handle(subscription);
  return jsubscription;
}

/*
 * nativeCreateClientHandle
 */
JNIEXPORT jlong
JNICALL Java_org_ros2_rcljava_Node_nativeCreateClientHandle(
    JNIEnv *env,
    jclass ,
     jlong jnode_handle,
    jclass jmessage_class,
   jstring jservice_topic,
   jobject jqos) {

  rcl_node_t *node = handle2Instance<rcl_node_t>(jnode_handle);
  rosidl_service_type_support_t *msg_type = jclass2ServiceType(env, jmessage_class);
  std::string service_topic = jstring2String(env, jservice_topic);

  rcl_client_t *client = makeInstance<rcl_client_t>();

  rcl_client_options_t client_ops = rcl_client_get_default_options();
  //  publisher_ops.qos =

  rcl_ret_t ret = rcl_client_init(
      client,
      node,
      msg_type,
      service_topic.c_str(),
      &client_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create client: " +
        std::string(rcl_get_error_string_safe()));
    throwException(env, message);

    return -1;
  }

  jlong jclient = instance2Handle(client);
  return jclient;
}

/*
 * nativeCreateServiceHandle
 */
JNIEXPORT jlong
JNICALL Java_org_ros2_rcljava_Node_nativeCreateServiceHandle(
    JNIEnv *env,
    jclass ,
     jlong jnode_handle,
    jclass jmessage_class,
   jstring jservice_topic,
   jobject jqos) {

  rcl_node_t *node = handle2Instance<rcl_node_t>(jnode_handle);
  rosidl_service_type_support_t *msg_type = jclass2ServiceType(env, jmessage_class);
  std::string service_topic = jstring2String(env, jservice_topic);

  rcl_service_t *service = makeInstance<rcl_service_t>();

  rcl_service_options_t service_ops = rcl_service_get_default_options();
  //  publisher_ops.qos =

  rcl_ret_t ret = rcl_service_init(
      service,
      node,
      msg_type,
      service_topic.c_str(),
      &service_ops);

  if (ret != RCL_RET_OK) {
    std::string message("Failed to create service: " +
        std::string(rcl_get_error_string_safe()));
    throwException(env, message);

    return -1;
  }

  jlong jservice = instance2Handle(service);
  return jservice;
}

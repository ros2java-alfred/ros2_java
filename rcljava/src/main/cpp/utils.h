/*
 * utils.h
 *
 *  Created on: 18 sept. 2016
 *      Author: mickael Gaillard <mick.gaillard@gmail.com>
 */

#ifndef SRC_RCL_UTILS_H_
#define SRC_RCL_UTILS_H_

#include <string>
#include <cstdlib>
#include <cassert>
#include <cstdio>
#include <jni.h>
#include <rcl/graph.h>
#include <rmw/rmw.h>

#include <rmw/types.h>
#include <rosidl_generator_c/message_type_support.h>

/*
 * Convert jstring to std::string.
 */
std::string
jstring2String(JNIEnv *env, jstring jsubject)
{
  const char *subject_tmp = env->GetStringUTFChars(jsubject, 0);
  std::string result(subject_tmp);
  env->ReleaseStringUTFChars(jsubject, subject_tmp);

  return result;
}

/*
 * Convertion of Message type.
 */
rosidl_message_type_support_t *
jclass2MessageType(JNIEnv *env, jclass jmessage_class)
{
  jmethodID mid =
      env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");

  jlong jts = env->CallStaticLongMethod(jmessage_class, mid);

  rosidl_message_type_support_t *ts =
      reinterpret_cast<rosidl_message_type_support_t *>(jts);

  return ts;
}

/*
 *
 */
void *
jclass2Message(JNIEnv *env, jclass jmessage_class)
{
  jmethodID jfrom_mid = env->GetStaticMethodID(jmessage_class, "getFromJavaConverter", "()J");
  jlong jfrom_java_converter = env->CallStaticLongMethod(jmessage_class, jfrom_mid);

  using convert_from_java_signature = void * (*)(jobject, void *);
  convert_from_java_signature convert_from_java =
      reinterpret_cast<convert_from_java_signature>(jfrom_java_converter);

  jmethodID jconstructor = env->GetMethodID(jmessage_class, "<init>", "()V");
  jobject jmsg = env->NewObject(jmessage_class, jconstructor);

  void *taken_msg = convert_from_java(jmsg, nullptr);
  return taken_msg;
}

jobject
jclass2JMessage(JNIEnv *env, jclass jmessage_class, void * taken_msg)
{
  jmethodID jto_mid = env->GetStaticMethodID(jmessage_class, "getToJavaConverter", "()J");
  jlong jto_java_converter = env->CallStaticLongMethod(jmessage_class, jto_mid);

  using convert_to_java_signature = jobject (*)(void *, jobject);
  convert_to_java_signature convert_to_java =
      reinterpret_cast<convert_to_java_signature>(jto_java_converter);

  jobject jtaken_msg = convert_to_java(taken_msg, nullptr);
  return jtaken_msg;
}

/*
 *
 */
void *
jobject2Message(JNIEnv *env, jobject jmessage)
{
  jclass jmessage_class = env->GetObjectClass(jmessage);
  jmethodID mid = env->GetStaticMethodID(jmessage_class, "getFromJavaConverter", "()J");
  jlong jfrom_java_converter = env->CallStaticLongMethod(jmessage_class, mid);

  typedef void * (* convert_from_java_signature)(jobject);
  convert_from_java_signature convert_from_java =
    reinterpret_cast<convert_from_java_signature>(jfrom_java_converter);

  void * raw_ros_message = convert_from_java(jmessage);
  return raw_ros_message;
}

/*
 *
 */
rosidl_service_type_support_t *
jclass2ServiceType(JNIEnv *env, jclass jmessage_class)
{
  jmethodID mid =
      env->GetStaticMethodID(jmessage_class, "getTypeSupport", "()J");

  jlong jts = env->CallStaticLongMethod(jmessage_class, mid);

  rosidl_service_type_support_t *ts =
      reinterpret_cast<rosidl_service_type_support_t *>(jts);

  return ts;
}

/*
 *
 */
template <typename T>
T *
makeInstance()
{
  return (T *)malloc(sizeof(T));
}

/*
 *
 */
template <typename T>
T*
handle2Instance(jlong handle)
{
  T * obj = reinterpret_cast<T *>(handle);

  assert(obj != NULL);

  return obj;
}

/*
 *
 */
jlong
instance2Handle(void* obj)
{
  jlong handler = reinterpret_cast<jlong>(obj);

  assert(handler != 0);

  return handler;
}

/*
 *
 */
void
throwException(JNIEnv *env, std::string message)
{
  const char *class_name = "java/lang/IllegalStateException";
  jclass exception_class = env->FindClass(class_name);

  assert(exception_class != NULL); // nullptr

  env->ThrowNew(exception_class, message.c_str());
}

/*
 *
 */
jobject
makeJTopics(JNIEnv *env, rcl_topic_names_and_types_t *topic_names_and_types)
{
  env->PushLocalFrame(256); // fix for local references

  jsize map_len   = topic_names_and_types->topic_count;
  jclass mapClass = env->FindClass("java/util/HashMap");
  jmethodID init  = env->GetMethodID(mapClass, "<init>", "(I)V");
  jobject hashMap = env->NewObject(mapClass, init, map_len);
  jmethodID put   = env->GetMethodID(mapClass, "put",
      "(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;");

  for (size_t i = 0; i < topic_names_and_types->topic_count; ++i) {
    env->CallObjectMethod(hashMap, put,
        env->NewStringUTF(topic_names_and_types->topic_names[i]),
        env->NewStringUTF(topic_names_and_types->type_names[i]));
  }

  env->PopLocalFrame(hashMap);

  return hashMap;
}

/*
jobject
makeJNames(rmw_ros_meta_t * ros_meta)
{
  java_util_ArrayList      = static_cast<jclass>(env->NewGlobalRef(env->FindClass("java/util/ArrayList")));
  java_util_ArrayList_     = env->GetMethodID(java_util_ArrayList, "<init>", "(I)V");
  java_util_ArrayList_size = env->GetMethodID (java_util_ArrayList, "size", "()I");
//  java_util_ArrayList_get  = env->GetMethodID(java_util_ArrayList, "get", "(I)Ljava/lang/Object;");
  java_util_ArrayList_add  = env->GetMethodID(java_util_ArrayList, "add", "(Ljava/lang/Object;)Z");

  jobject result = env->NewObject(java_util_ArrayList, java_util_ArrayList_, vector.size());

  for (size_t i = 0; i < rmw_ros_meta_t->topic_count; ++i) {
    jstring element = env->NewStringUTF(ros_meta_data->node_names[i].c_str());

    env->CallBooleanMethod(result, java_util_ArrayList_add, element);
    env->DeleteLocalRef(element);
  }
  return result;
}
*/

#endif /* SRC_RCL_UTILS_H_ */

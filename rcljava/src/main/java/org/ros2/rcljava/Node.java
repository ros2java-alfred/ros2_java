/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros2.rcljava;

import java.lang.ref.WeakReference;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * This class serves as a bridge between ROS2's rcl_node_t and RCLJava.
 * A Node must be created via @{link RCLJava#createNode(String)}
 */
public class Node {
  static {
    try {
      System.loadLibrary("rcljavaNode__" + RCLJava.getRMWIdentifier());
    } catch (UnsatisfiedLinkError ule) {
      System.err.println("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  /**
   * An integer that represents a pointer to the underlying ROS2 node
   * structure (rcl_node_t).
   */
  private final long nodeHandle;

  /**
   * All the @{link Subscription}s that have been created through this instance.
   */
  private final Queue<Subscription> subscriptions;

  /**
   * All the @{link Publisher}s that have been created through this instance.
   */
  private final Queue<Publisher> publishers;

  /**
   * All the @{link Service}s that have been created through this instance.
   */
  private final Queue<Service> services;

  /**
   * All the @{link Client}s that have been created through this instance.
   */
  private final Queue<Client> clients;

  /**
   * Constructor.
   *
   * @param nodeHandle A pointer to the underlying ROS2 node structure. Must not
   *     be zero.
   */
  public Node(final long nodeHandle) {
    this.nodeHandle = nodeHandle;
    this.publishers = new LinkedBlockingQueue<Publisher>();
    this.subscriptions = new LinkedBlockingQueue<Subscription>();
    this.services = new LinkedBlockingQueue<Service>();
    this.clients = new LinkedBlockingQueue<Client>();
  }

  /**
   * Create a ROS2 publisher (rcl_publisher_t) and return a pointer to
   *     it as an integer.
   *
   * @param <T> The type of the messages that will be published by the
   *     created @{link Publisher}.
   * @param nodeHandle A pointer to the underlying ROS2 node structure.
   * @param messageType The class of the messages that will be published by the
   *     created @{link Publisher}.
   * @param topic The topic to which the created @{link Publisher} will
   *     publish messages.
   * @return A pointer to the underlying ROS2 publisher structure.
   */
  private static native <T> long nativeCreatePublisherHandle(
      long nodeHandle, Class<T> messageType, String topic);

  /**
   * Create a ROS2 subscription (rcl_subscription_t) and return a pointer to
   *     it as an integer.
   *
   * @param <T> The type of the messages that will be received by the
   *     created @{link Subscription}.
   * @param nodeHandle A pointer to the underlying ROS2 node structure.
   * @param messageType The class of the messages that will be received by the
   *     created @{link Subscription}.
   * @param topic The topic from which the created @{link Subscription} will
   *     receive messages.
   * @return A pointer to the underlying ROS2 subscription structure.
   */
  private static native <T> long nativeCreateSubscriptionHandle(
      long nodeHandle, Class<T> messageType, String topic);

  /**
   * Create a Publisher&lt;T&gt;.
   *
   * @param <T> The type of the messages that will be published by the
   *     created @{link Publisher}.
   * @param messageType The class of the messages that will be published by the
   *     created @{link Publisher}.
   * @param topic The topic to which the created @{link Publisher} will
   *     publish messages.
   * @return A @{link Publisher} that represents the underlying ROS2 publisher
   *     structure.
   */
  public final <T> Publisher<T> createPublisher(
      final Class<T> messageType, final String topic) {
    long publisherHandle = nativeCreatePublisherHandle(this.nodeHandle,
        messageType, topic);
    Publisher<T> publisher = new Publisher<T>(this.nodeHandle,
        publisherHandle, topic);
    this.publishers.add(publisher);
    return publisher;
  }

  /**
   * Create a Subscription&lt;T&gt;.
   *
   * @param <T> The type of the messages that will be received by the
   *     created @{link Subscription}.
   * @param messageType The class of the messages that will be received by the
   *     created @{link Subscription}.
   * @param topic The topic from which the created @{link Subscription} will
   *     receive messages.
   * @param callback The callback function that will be triggered when a
   *     message is received by the @{link Subscription}.
   * @return A @{link Subscription} that represents the underlying ROS2
   *     subscription structure.
   */
  public final <T> Subscription<T> createSubscription(
      final Class<T> messageType, final String topic,
      final Consumer<T> callback) {

    long subscriptionHandle = nativeCreateSubscriptionHandle(
        this.nodeHandle, messageType, topic);

    Subscription<T> subscription = new Subscription<T>(
        this.nodeHandle, subscriptionHandle, messageType, topic, callback);

    this.subscriptions.add(subscription);
    return subscription;
  }

  /**
   * @return All the @{link Subscription}s that were created by this instance.
   */
  public final Queue<Subscription> getSubscriptions() {
    return this.subscriptions;
  }

  /**
   * @return All the @{link Publisher}s that were created by this instance.
   */
  public final Queue<Publisher> getPublishers() {
    return this.publishers;
  }

  /**
   * Safely destroy the underlying ROS2 node structure.
   */
  public final void dispose() {
    // TODO(esteve): implement
  }

  public final long getNodeHandle() {
    return this.nodeHandle;
  }

  private static native <T> long nativeCreateServiceHandle(
      long nodeHandle, Class<T> cls, String serviceName);

  public final <T> Service<T> createService(final Class<T> serviceType,
      final String serviceName, final TriConsumer<RMWRequestId, ?, ?> callback)
      throws NoSuchFieldException, IllegalAccessException,
      NoSuchMethodException, InvocationTargetException {

    Class requestType = (Class) serviceType.getField("RequestType").get(null);

    Method requestFromJavaConverterMethod = requestType.getDeclaredMethod(
        "getFromJavaConverter", (Class<?>[]) null);
    long requestFromJavaConverterHandle =
        (Long) requestFromJavaConverterMethod.invoke(null, (Class<?>[]) null);

    Method requestToJavaConverterMethod = requestType.getDeclaredMethod(
        "getToJavaConverter", (Class<?>[]) null);
    long requestToJavaConverterHandle =
        (Long) requestToJavaConverterMethod.invoke(null, (Class<?>[]) null);

    Class responseType = (Class) serviceType.getField("ResponseType").get(null);

    Method responseFromJavaConverterMethod = responseType.getDeclaredMethod(
        "getFromJavaConverter", (Class<?>[]) null);
    long responseFromJavaConverterHandle =
        (Long) responseFromJavaConverterMethod.invoke(null, (Class<?>[]) null);

    Method responseToJavaConverterMethod = responseType.getDeclaredMethod(
        "getToJavaConverter", (Class<?>[]) null);
    long responseToJavaConverterHandle =
        (Long) responseToJavaConverterMethod.invoke(null, (Class<?>[]) null);

    long serviceHandle = nativeCreateServiceHandle(this.nodeHandle,
        serviceType, serviceName);
    Service<T> service = new Service<T>(this.nodeHandle, serviceHandle,
        serviceType, serviceName,  callback, requestType, responseType,
        requestFromJavaConverterHandle, requestToJavaConverterHandle,
        responseFromJavaConverterHandle, responseToJavaConverterHandle);
    this.services.add(service);
    return service;
  }

  public final Queue<Service> getServices() {
    return this.services;
  }

  public final <T> Client<T> createClient(final Class<T> serviceType,
      final String serviceName) throws NoSuchFieldException,
      IllegalAccessException, NoSuchMethodException, InvocationTargetException {

    Class requestType = (Class) serviceType.getField("RequestType").get(null);

    Method requestFromJavaConverterMethod = requestType.getDeclaredMethod(
        "getFromJavaConverter", (Class<?>[]) null);
    long requestFromJavaConverterHandle =
        (Long) requestFromJavaConverterMethod.invoke(null, (Class<?>[]) null);

    Method requestToJavaConverterMethod = requestType.getDeclaredMethod(
        "getToJavaConverter", (Class<?>[]) null);
    long requestToJavaConverterHandle =
        (Long) requestToJavaConverterMethod.invoke(null, (Class<?>[]) null);

    Class responseType = (Class) serviceType.getField("ResponseType").get(null);

    Method responseFromJavaConverterMethod = responseType.getDeclaredMethod(
        "getFromJavaConverter", (Class<?>[]) null);
    long responseFromJavaConverterHandle =
        (Long) responseFromJavaConverterMethod.invoke(null, (Class<?>[]) null);

    Method responseToJavaConverterMethod = responseType.getDeclaredMethod(
        "getToJavaConverter", (Class<?>[]) null);
    long responseToJavaConverterHandle =
        (Long) responseToJavaConverterMethod.invoke(null, (Class<?>[]) null);

    long clientHandle = nativeCreateClientHandle(this.nodeHandle, serviceType,
        serviceName);
    Client<T> client = new Client<T>(new WeakReference<Node>(this),
        this.nodeHandle, clientHandle, serviceType, serviceName, requestType,
        responseType, requestFromJavaConverterHandle,
        requestToJavaConverterHandle, responseFromJavaConverterHandle,
        responseToJavaConverterHandle);
    this.clients.add(client);
    return client;
  }

  private static native <T> long nativeCreateClientHandle(
      long nodeHandle, Class<T> cls, String serviceName);

  public final Queue<Client> getClients() {
    return this.clients;
  }
}

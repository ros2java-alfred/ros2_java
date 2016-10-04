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

import java.lang.reflect.Method;

import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Entry point for the ROS2 Java API, similar to the rclcpp API.
 */
public final class RCLJava {
  /**
   * Private constructor so this cannot be instantiated.
   */
  private RCLJava() { }

  /**
   * All the @{link Node}s that have been created.
   */
  private static Queue<Node> nodes;

  static {
    nodes = new LinkedBlockingQueue<Node>();

    RMW_TO_TYPESUPPORT = new ConcurrentSkipListMap<String, String>() {{
        put("rmw_fastrtps_cpp", "rosidl_typesupport_introspection_c");
        put("rmw_opensplice_cpp", "rosidl_typesupport_opensplice_c");
        put("rmw_connext_cpp", "rosidl_typesupport_connext_c");
        put("rmw_connext_dynamic_cpp", "rosidl_typesupport_introspection_c");
      }
    };

    Runtime.getRuntime().addShutdownHook(new Thread() {
      public void run() {
        for (Node node : nodes) {
          node.dispose();
        }
      }
    });
  }

  /**
   * The identifier of the currently active RMW implementation.
   */
  private static String rmwImplementation = null;

  /**
   * Flag to indicate if RCLJava has been fully initialized, with a valid RMW
   *   implementation.
   */
  private static boolean initialized = false;

  /**
   * A mapping between RMW implementations and their typesupports.
   */
  private static final Map<String, String> RMW_TO_TYPESUPPORT;

  /**
   * @return true if RCLJava has been fully initialized, false otherwise.
   */
  public static boolean isInitialized() {
    return RCLJava.initialized;
  }

  /**
   * Initialize the RCLJava API. If successful, a valid RMW implementation will
   *   be loaded and accessible, enabling the creating of ROS2 entities
   *   (@{link Node}s, @{link Publisher}s and @{link Subscription}s.
   */
  public static void rclJavaInit() {
    synchronized (RCLJava.class) {
      if (!initialized) {
        if (RCLJava.rmwImplementation == null) {
          for (Map.Entry<String, String> entry
               : RMW_TO_TYPESUPPORT.entrySet()) {

            try {
              setRMWImplementation(entry.getKey());
              break;
            } catch (UnsatisfiedLinkError ule) {
              // TODO(esteve): handle exception
            } catch (Exception exc) {
              // TODO(esteve): handle exception
            }
          }
        }
        if (RCLJava.rmwImplementation == null) {
          System.err.println("No RMW implementation found");
          System.exit(1);
        } else {
          nativeRCLJavaInit();
          System.out.println("Using RMW implementation: " + getRMWIdentifier());
          initialized = true;
        }
      }
    }
  }

  /**
   * Initialize the underlying rcl layer.
   */
  private static native void nativeRCLJavaInit();

  /**
   * Create a ROS2 node (rcl_node_t) and return a pointer to it as an integer.
   *
   * @param nodeName The name that will identify this node in a ROS2 graph.
   * @return A pointer to the underlying ROS2 node structure.
   */
  private static native long nativeCreateNodeHandle(String nodeName);

  public static String getTypesupportIdentifier() {
    return RMW_TO_TYPESUPPORT.get(nativeGetRMWIdentifier());
  }

  public static void setRMWImplementation(
      final String rmwImplementation) throws Exception {

    synchronized (RCLJava.class) {
      System.loadLibrary("rcljavaRCLJava__" + rmwImplementation);
      RCLJava.rmwImplementation = rmwImplementation;
    }
  }

  /**
   * @return The identifier of the currently active RMW implementation via the
   *     native ROS2 API.
   */
  private static native String nativeGetRMWIdentifier();

  /**
   * @return The identifier of the currently active RMW implementation.
   */
  public static String getRMWIdentifier() {
    return nativeGetRMWIdentifier();
  }

  /**
   * Call the underlying ROS2 rcl mechanism to check if ROS2 has been shut
   *   down.
   *
   * @return true if RCLJava hasn't been shut down, false otherwise.
   */
  private static native boolean nativeOk();

  /**
   * @return true if RCLJava hasn't been shut down, false otherwise.
   */
  public static boolean ok() {
    return nativeOk();
  }

  /**
   * Create a @{link Node}.
   *
   * @param nodeName The name that will identify this node in a ROS2 graph.
   * @return A @{link Node} that represents the underlying ROS2 node
   *     structure.
   */
  public static Node createNode(final String nodeName) {
    long nodeHandle = nativeCreateNodeHandle(nodeName);
    Node node = new Node(nodeHandle);
    nodes.add(node);
    return node;
  }

  public static void spinOnce(final Node node) {
    long waitSetHandle = nativeGetZeroInitializedWaitSet();

    nativeWaitSetInit(waitSetHandle, node.getSubscriptions().size(), 0, 0, node.getClients().size(), node.getServices().size());

    nativeWaitSetClearSubscriptions(waitSetHandle);

    nativeWaitSetClearServices(waitSetHandle);

    nativeWaitSetClearClients(waitSetHandle);

    for (Subscription subscription : node.getSubscriptions()) {
      nativeWaitSetAddSubscription(
          waitSetHandle, subscription.getSubscriptionHandle());
    }

    for (Service service : node.getServices()) {
      nativeWaitSetAddService(waitSetHandle, service.getServiceHandle());
    }

    for (Client client : node.getClients()) {
      nativeWaitSetAddClient(waitSetHandle, client.getClientHandle());
    }

    nativeWait(waitSetHandle);

    for (Subscription subscription : node.getSubscriptions()) {
      Object message = nativeTake(
          subscription.getSubscriptionHandle(),
          subscription.getMessageType());
      if (message != null) {
        subscription.getCallback().accept(message);
      }
    }

    for (Service service : node.getServices()) {
      long requestFromJavaConverterHandle = service.getRequestFromJavaConverterHandle();
      long requestToJavaConverterHandle = service.getRequestToJavaConverterHandle();
      long responseFromJavaConverterHandle = service.getResponseFromJavaConverterHandle();
      long responseToJavaConverterHandle = service.getResponseToJavaConverterHandle();

      Class requestType = service.getRequestType();
      Class responseType = service.getResponseType();

      Object requestMessage = null;
      Object responseMessage = null;

      try {
        requestMessage = requestType.newInstance();
        responseMessage = responseType.newInstance();
      } catch (InstantiationException ie) {
        ie.printStackTrace();
        continue;
      } catch (IllegalAccessException iae) {
        iae.printStackTrace();
        continue;
      }

      RMWRequestId rmwRequestId = (RMWRequestId)nativeTakeRequest(service.getServiceHandle(), requestFromJavaConverterHandle, requestToJavaConverterHandle, requestMessage);
      if (rmwRequestId != null) {
        service.getCallback().accept(rmwRequestId, requestMessage, responseMessage);
        nativeSendServiceResponse(service.getServiceHandle(), rmwRequestId, responseFromJavaConverterHandle, responseToJavaConverterHandle, responseMessage);
      }
    }

    for (Client client : node.getClients()) {
      long requestFromJavaConverterHandle = client.getRequestFromJavaConverterHandle();
      long requestToJavaConverterHandle = client.getRequestToJavaConverterHandle();
      long responseFromJavaConverterHandle = client.getResponseFromJavaConverterHandle();
      long responseToJavaConverterHandle = client.getResponseToJavaConverterHandle();

      Class requestType = client.getRequestType();
      Class responseType = client.getResponseType();

      Object requestMessage = null;
      Object responseMessage = null;

      try {
        requestMessage = requestType.newInstance();
        responseMessage = responseType.newInstance();
      } catch (InstantiationException ie) {
        ie.printStackTrace();
        continue;
      } catch (IllegalAccessException iae) {
        iae.printStackTrace();
        continue;
      }

      RMWRequestId rmwRequestId = (RMWRequestId)nativeTakeResponse(
          client.getClientHandle(), responseFromJavaConverterHandle,
          responseToJavaConverterHandle, responseMessage);

      if (rmwRequestId != null) {
        client.handleResponse(rmwRequestId, responseMessage);
      }
    }
  }

  private static native void nativeShutdown();

  public static void shutdown() {
    nativeShutdown();
  }

  private static native long nativeGetZeroInitializedWaitSet();

  private static native void nativeWaitSetInit(
      long waitSetHandle, int numberOfSubscriptions,
      int numberOfGuardConditions, int numberOfTimers,
      int numberOfClients, int numberOfServices);

  private static native void nativeWaitSetClearSubscriptions(
      long waitSetHandle);

  private static native void nativeWaitSetAddSubscription(
      long waitSetHandle, long subscriptionHandle);

  private static native void nativeWait(long waitSetHandle);

  private static native Object nativeTake(long subscriptionHandle,
      Class messageType);

  private static native void nativeWaitSetClearServices(long waitSetHandle);

  private static native void nativeWaitSetAddService(long waitSetHandle, long serviceHandle);

  private static native void nativeWaitSetClearClients(long waitSetHandle);

  private static native void nativeWaitSetAddClient(long waitSetHandle, long clientHandle);

  private static native Object nativeTakeRequest(
      long serviceHandle, long requestFromJavaConverterHandle,
      long requestToJavaConverterHandle, Object requestMessage);

  private static native void nativeSendServiceResponse(long serviceHandle, Object header,
      long responseFromJavaConverterHandle, long responseToJavaConverterHandle,
      Object responseMessage);

  private static native Object nativeTakeResponse(
      long clientHandle, long responseFromJavaConverterHandle,
      long responseToJavaConverterHandle, Object responseMessage);
}

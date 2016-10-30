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

import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.LinkedBlockingQueue;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.ros2.rcljava.exception.ImplementationAlreadyImportedException;
import org.ros2.rcljava.exception.NoImplementationAvailableException;
import org.ros2.rcljava.exception.NotInitializedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.qos.QoSProfile;

/**
 * ROS2 java client wrapper.
 *
 * <p>JNI call of ROS2 c client.</p>
 *
 * @author Esteve Fernandez <esteve@apache.org>
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class RCLJava {

    private static final String LOG_NAME = RCLJava.class.getName();

    private static final Logger logger = LoggerFactory.getLogger(RCLJava.class);

    /** Global List/Queue of publishers. */
    public static Queue<WeakReference<Publisher<?>>> publisherReferences =
            new LinkedBlockingQueue<WeakReference<Publisher<?>>>();

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
     * All the @{link Node}s that have been created.
     */
    private static Queue<Node> nodes = new LinkedBlockingQueue<Node>();

    /**
     * A mapping between RMW implementations and their typesupports.
     */
    private static final Map<String, String> RMW_TO_TYPESUPPORT = new ConcurrentSkipListMap<String, String>() {
        /** Serial Id */
        private static final long serialVersionUID = 1L;

        {
            put("rmw_fastrtps_cpp",         "rosidl_typesupport_introspection_c");
            put("rmw_opensplice_cpp",       "rosidl_typesupport_opensplice_c");
            put("rmw_connext_cpp",          "rosidl_typesupport_connext_c");
            put("rmw_connext_dynamic_cpp",  "rosidl_typesupport_introspection_c");
        }
    };

    // Natives definitions
    //################################################################################################################//

    // rcl.h
    /**
     * Initialize the underlying rcl layer.
     */
    private static native void nativeRCLJavaInit(String args[]);

    private static native void nativeShutdown();

    /**
     * Call the underlying ROS2 rcl mechanism to check if ROS2 has been shut
     *   down.
     *
     * @return true if RCLJava hasn't been shut down, false otherwise.
     */
    private static native boolean nativeOk();

    /**
     * @return The identifier of the currently active RMW implementation via the
     *     native ROS2 API.
     */
    private static native String nativeGetRMWIdentifier();

    // Node.h
    /**
     * Create a ROS2 node (rcl_node_t) and return a pointer to it as an integer.
     *
     * @param nodeName The name that will identify this node in a ROS2 graph.
     * @return A pointer to the underlying ROS2 node structure.
     */
    private static native long nativeCreateNodeHandle(String nodeName);

    // Wait.h
    private static native long nativeGetZeroInitializedWaitSet();
    private static native void nativeWaitSetInit(
            long waitSetHandle,
            int numberOfSubscriptions,
            int numberOfGuardConditions,
            int numberOfTimers,
            int numberOfClients,
            int numberOfServices);
    private static native void nativeWaitSetClearSubscriptions(long waitSetHandle);
    private static native void nativeWaitSetAddSubscription(long waitSetHandle, long subscriptionHandle);
    private static native void nativeWaitSetClearServices(long waitSetHandle);
    private static native void nativeWaitSetAddService(long waitSetHandle, long serviceHandle);
    private static native void nativeWaitSetClearClients(long waitSetHandle);
    private static native void nativeWaitSetAddClient(long waitSetHandle, long clientHandle);
    private static native void nativeWait(long waitSetHandle);
    private static native Message nativeTake(long SubscriptionHandle, Class<?> msgType);
    private static native void nativeWaitSetFini(long waitSetHandle);
    private static native List<String> nativeGetNodeNames();
    private static native Object nativeTakeRequest(
            long serviceHandle,
            long requestFromJavaConverterHandle,
            long requestToJavaConverterHandle,
            Object requestMessage);
    private static native void nativeSendServiceResponse(
            long serviceHandle,
            Object header,
            long responseFromJavaConverterHandle,
            long responseToJavaConverterHandle,
            Object responseMessage);
    private static native Object nativeTakeResponse(
            long clientHandle,
            long responseFromJavaConverterHandle,
            long responseToJavaConverterHandle,
            Object responseMessage);
    private static native long nativeConvertQoSProfileToHandle(
            int history, int depth, int reliability, int durability);
    private static native void nativeDisposeQoSProfile(
            long qosProfileHandle);

    /** Release all ressources at shutdown. */
    static {
        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                if (RCLJava.initialized) {
                    RCLJava.logger.debug("Final Shutdown...");

                    String[] list = NativeUtils.getLoadedLibraries(RCLJava.class.getClassLoader());
                    StringBuilder msgLog = new StringBuilder();
                    for (String key : list) {
                        msgLog.append(key);
                        msgLog.append("\n");
                    }
                    RCLJava.logger.debug("Native libraries Loaded: \n" + msgLog.toString());

                    for(WeakReference<Publisher<?>> publisherReference : RCLJava.publisherReferences) {
                        if(publisherReference.get() != null) {
                            publisherReference.get().dispose();
                        }
                    }
                }

                for (Node node : nodes) {
                    node.dispose();
                }
            }
        });
    }

    /**
     * Private constructor so this cannot be instantiated.
     */
    private RCLJava() { }

    /**
     * Initialize the RCLJava API. If successful, a valid RMW implementation will
     *   be loaded and accessible, enabling the creating of ROS2 entities
     *   (@{link Node}s, @{link Publisher}s and @{link Subscription}s.
     */
    public static void rclJavaInit(final String args[]) {
        synchronized (RCLJava.class) {
            if (!RCLJava.initialized) {
                if (RCLJava.rmwImplementation == null) {
                    String libpath = System.getProperty("java.library.path");
                    RCLJava.logger.debug("Native Library path : \n" + libpath.replace(':', '\n'));

                    RCLJava.autoLoadRmw();
                }

                if (RCLJava.rmwImplementation == null) {
                    RCLJava.logger.error("No RMW implementation found...");
                    System.exit(1);
                } else {
                    RCLJava.logger.debug("Initialize rclJava with " + RCLJava.rmwImplementation);
                    RCLJava.nativeRCLJavaInit(args);
                    RCLJava.initialized = true;
                }
            } else {
                NotInitializedException ex = new NotInitializedException("Cannot intialized twice !");
                logger.error(ex.getMessage());
                throw ex;
            }
        }
    }

    /**
     * @return true if RCLJava has been fully initialized, false otherwise.
     */
    public static boolean isInitialized() {
        return RCLJava.initialized;
    }

    public static void rclJavaInit() {
        RCLJava.rclJavaInit(null);
    }

    /**
     * Create a @{link Node}.
     *
     * @param nodeName Name of the node.
     * @param nodeName The name that will identify this node in a ROS2 graph.
     * @return A @{link Node} that represents the underlying ROS2 node
     *     structure.
     */
    public static Node createNode(final String nodeName) {
        RCLJava.logger.debug("Create Node stack : " + nodeName);

        if (!RCLJava.initialized) {
            throw new NotInitializedException();
        }

        long nodeHandle = RCLJava.nativeCreateNodeHandle(nodeName);
        Node node = new Node(nodeHandle, nodeName);
        nodes.add(node);

        return node;
    }


    /**
     * Wait for once loop.
     *
     * @param node
     */
    public static void spinOnce(final Node node) {
        if (!RCLJava.initialized) {
            throw new NotInitializedException();
        }

        if (node.getClients().size() > 0 ||
                node.getPublishers().size() > 0 ||
                node.getServices().size() > 0 ||
                node.getSubscriptions().size() > 0) {
            long waitSetHandle = RCLJava.nativeGetZeroInitializedWaitSet();

            RCLJava.nativeWaitSetInit(
                    waitSetHandle,
                    node.getSubscriptions().size(),
                    0,
                    0,
                    node.getClients().size(),
                    node.getServices().size());

            RCLJava.nativeWaitSetClearSubscriptions(waitSetHandle);
            RCLJava.nativeWaitSetClearServices(waitSetHandle);
            RCLJava.nativeWaitSetClearClients(waitSetHandle);

            for (Subscription<?> subscription : node.getSubscriptions()) {
                RCLJava.nativeWaitSetAddSubscription(waitSetHandle, subscription.getSubscriptionHandle());
            }

            for (Service<?> service : node.getServices()) {
                RCLJava.nativeWaitSetAddService(waitSetHandle, service.getServiceHandle());
            }

            for (Client<?> client : node.getClients()) {
                RCLJava.nativeWaitSetAddClient(waitSetHandle, client.getClientHandle());
            }

            RCLJava.nativeWait(waitSetHandle);
            RCLJava.nativeWaitSetFini(waitSetHandle);

            for (Subscription subscription : node.getSubscriptions()) {
                Message message = RCLJava.nativeTake(subscription.getSubscriptionHandle(), subscription.getMessageType());
                if (message != null) {
                    subscription.getCallback().accept(message);
                }
            }

            for (Service service : node.getServices()) {
                long requestFromJavaConverterHandle = service.getRequestFromJavaConverterHandle();
                long requestToJavaConverterHandle = service.getRequestToJavaConverterHandle();
                long responseFromJavaConverterHandle = service.getResponseFromJavaConverterHandle();
                long responseToJavaConverterHandle = service.getResponseToJavaConverterHandle();

                Class<?> requestType = service.getRequestType();
                Class<?> responseType = service.getResponseType();

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

                RMWRequestId rmwRequestId = (RMWRequestId) RCLJava.nativeTakeRequest(
                        service.getServiceHandle(),
                        requestFromJavaConverterHandle,
                        requestToJavaConverterHandle,
                        requestMessage);

                if (rmwRequestId != null) {
                    service.getCallback().accept(rmwRequestId, requestMessage, responseMessage);
                    RCLJava.nativeSendServiceResponse(service.getServiceHandle(), rmwRequestId, responseFromJavaConverterHandle,
                            responseToJavaConverterHandle, responseMessage);
                }
            }

            for (Client<?> client : node.getClients()) {
                long responseFromJavaConverterHandle = client.getResponseFromJavaConverterHandle();
                long responseToJavaConverterHandle = client.getResponseToJavaConverterHandle();

                Class<?> responseType = client.getResponseType();

                Object responseMessage = null;

                try {
                    responseMessage = responseType.newInstance();
                } catch (InstantiationException ie) {
                    ie.printStackTrace();
                    continue;
                } catch (IllegalAccessException iae) {
                    iae.printStackTrace();
                    continue;
                }

                RMWRequestId rmwRequestId = (RMWRequestId) RCLJava.nativeTakeResponse(
                        client.getClientHandle(),
                        responseFromJavaConverterHandle,
                        responseToJavaConverterHandle,
                        responseMessage);


                if (rmwRequestId != null) {
                    client.handleResponse(rmwRequestId, responseMessage);
                }
            }
        } else {
            // TODO fix to rate sleep.
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Helper Spin.
     * @param node
     */
    public static void spin(final Node node) {
        while(RCLJava.ok()) {
            RCLJava.spinOnce(node);
        }
    }

    /**
     *
     * @param node
     * @param topic
     * @return
     */
    public static Message waitForMessage(final Node node, final String topic) {
        if (!RCLJava.initialized) {
            throw new NotInitializedException();
        }

        Message msg = null;
        long waitSetHandle = RCLJava.nativeGetZeroInitializedWaitSet();

        RCLJava.nativeWaitSetInit(waitSetHandle, 1, 0, 0, 0, 0);

        RCLJava.nativeWaitSetClearSubscriptions(waitSetHandle);
        for(Subscription<?> subscription : node.getSubscriptions()) {
            if (subscription.getTopic() == topic) {
                RCLJava.nativeWaitSetAddSubscription(waitSetHandle, subscription.getSubscriptionHandle());
            }
        }

        RCLJava.nativeWait(waitSetHandle);

        for(Subscription<?> subscription : node.getSubscriptions()) {
            if (subscription.getTopic() == topic) {
                msg = RCLJava.nativeTake(subscription.getSubscriptionHandle(), subscription.getMessageType());
                if (msg != null) {
                    break;
                }
            }
        }

        RCLJava.nativeWaitSetFini(waitSetHandle);

        return msg;

    }

    /**
     * Return true if rcl is currently initialized, otherwise false.
     *
     * @see rcl_ok();
     *
     * @return true if RCLJava hasn't been shut down, false otherwise.
     */
    public static boolean ok() {
        if (!RCLJava.initialized) {
            throw new NotInitializedException();
        }

        return RCLJava.nativeOk();
    }

    /**
     * Signal global shutdown of RCLJava.
     *
     * <p>This function does not have to be called on exit, but does have to be called making a
     * repeat call to RCLJava.rclJavaInit.</p>
     *
     * <p>This function can only be called once after each call to RCLJava.rclJavaInit.</p>
     */
    public static void shutdown() {
        RCLJava.logger.debug("Shutdown...");

        if (!RCLJava.initialized) {
            throw new NotInitializedException();
        }

        RCLJava.nativeShutdown();
        RCLJava.initialized = false;
    }

    /**
     * @return The identifier of the currently active RMW implementation.
     */
    public static String getRMWIdentifier() {
        if (!RCLJava.initialized) {
            throw new NotInitializedException();
        }

        return RCLJava.nativeGetRMWIdentifier();
    }

    public static List<String> getNodeNames() {
        return RCLJava.nativeGetNodeNames();
    }

    /**
     * Get identifier of the ROS2 middleware use.
     *
     * <p>TODO rename to list of RMW available.</p>
     *
     * @return Identifier string of ROS2 middleware.
     */
    public static String getTypesupportIdentifier() {
        String typesupportIdentifier = RMW_TO_TYPESUPPORT.get(RCLJava.getRMWIdentifier());

        return typesupportIdentifier;
    }

    /**
     * Switch of ROS2 middleware implementation
     *
     * <p>TODO need to check implementation available.</p>
     *
     * @param rmwImplementation
     * @throws NoImplementationAvailableException
     */
    public static void setRMWImplementation(String rmwImplementation)
            throws NoImplementationAvailableException, ImplementationAlreadyImportedException {

        synchronized(RCLJava.class) {
            if (rmwImplementation != null && !rmwImplementation.isEmpty()) {
                if (!rmwImplementation.equals(RCLJava.rmwImplementation)) {

                    String file = "rcljavaRCLJava__" + rmwImplementation;
                    RCLJava.logger.debug("Load native RMW file : lib" + file + ".so");

                    try {
                        System.loadLibrary(file);
                        RCLJava.rmwImplementation = rmwImplementation;
                    } catch (UnsatisfiedLinkError e) {
                        throw new NoImplementationAvailableException(e);
                    } catch (Exception e) {
                        throw new NoImplementationAvailableException(e);
                    }
                } else {
                    throw new ImplementationAlreadyImportedException();
                }
            } else {
                RCLJava.logger.debug("Disable RMW !");
                RCLJava.rmwImplementation = null;
            }
        }
    }

    /**
     * <h1>Load Native ROS library</h1>
     * <p>load from java.library.path .</p>
     * @param name Name of the library.
     */
    public static void loadLibrary(String name) {
        synchronized(RCLJava.class) {
            RCLJava.logger.info("Load native file : lib" + name + ".so");

            if (!RCLJava.initialized) {
                throw new NotInitializedException();
            }

            try {
                System.loadLibrary(name);
            } catch (UnsatisfiedLinkError e) {
                RCLJava.logger.error("Native code library failed to load.", e);
                System.exit(1);
            }
        }
    }

    /**
     * Load RMW.
     */
    private static void autoLoadRmw() {
        for (Map.Entry<String, String> entry : RMW_TO_TYPESUPPORT.entrySet()) {
            try {
                RCLJava.logger.info("Try to load native " + entry.getKey() + "...");
                RCLJava.setRMWImplementation(entry.getKey());
                RCLJava.logger.info(entry.getKey() + " loaded !");
                break;
            } catch (NoImplementationAvailableException e) {
                RCLJava.logger.error(entry.getKey() + " not available ! (" + e.getMessage() + ")");
            } catch (ImplementationAlreadyImportedException e) {
                RCLJava.logger.error(e.getMessage());
            }
        }
    }


    public static long convertQoSProfileToHandle(final QoSProfile qosProfile) {
      int history = qosProfile.getHistory().getValue();
      int depth = qosProfile.getDepth();
      int reliability = qosProfile.getReliability().getValue();
      int durability = qosProfile.getDurability().getValue();

      RCLJava.logger.debug("Convert QosProfile...");

      return nativeConvertQoSProfileToHandle(history, depth, reliability,
        durability);
    }

    public static void disposeQoSProfile(final long qosProfileHandle) {
        nativeDisposeQoSProfile(qosProfileHandle);
    }

}

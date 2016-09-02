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

import java.util.Queue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import java.util.Map;
import java.util.concurrent.ConcurrentSkipListMap;

public class RCLJava {
    public static Queue<WeakReference> publisherReferences = new LinkedBlockingQueue();

    static {
        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                for(WeakReference<Publisher> publisherReference : publisherReferences) {
                    if(publisherReference.get() != null) {
                        publisherReference.get().dispose();
                    }
                }
            }
        });
    }

    private static String rmwImplementation = null;
    private static boolean initialized = false;

    private static final Map<String, String> rmwToTypesupport = new ConcurrentSkipListMap<String, String>() {{
        put("rmw_fastrtps_cpp", "rosidl_typesupport_introspection_c");
        put("rmw_opensplice_cpp", "rosidl_typesupport_opensplice_c");
        put("rmw_connext_cpp", "rosidl_typesupport_connext_c");
        put("rmw_connext_dynamic_cpp", "rosidl_typesupport_introspection_c");
    }};

    public static void rclJavaInit() {
        synchronized(RCLJava.class) {
            if (!initialized) {
                if (RCLJava.rmwImplementation == null) {
                    for(Map.Entry<String, String> entry : rmwToTypesupport.entrySet()) {
                        try {
                            setRMWImplementation(entry.getKey());
                            break;
                        } catch(UnsatisfiedLinkError ule) {
                            // TODO(esteve): handle exception
                        } catch(Exception e) {
                            // TODO(esteve): handle exception
                        }
                    }
                }
                if (RCLJava.rmwImplementation == null) {
                    System.err.println("No RMW implementation found");
                    System.exit(1);
                } else {
                    nativeRCLJavaInit();
                    initialized = true;
                }
            }
        }
    }

    private static native void nativeRCLJavaInit();

    private static native long nativeCreateNodeHandle(String nodeName);

    public static String getTypesupportIdentifier() {
        String typesupportIdentifier = rmwToTypesupport.get(nativeGetRMWIdentifier());
        return typesupportIdentifier;
    }

    public static void setRMWImplementation(String rmwImplementation) throws Exception {
        synchronized(RCLJava.class) {
            System.loadLibrary("rcljavaRCLJava__" + rmwImplementation);
            RCLJava.rmwImplementation = rmwImplementation;
        }
    }

    private static native String nativeGetRMWIdentifier();

    public static String getRMWIdentifier() {
        return nativeGetRMWIdentifier();
    }

    private static native boolean nativeOk();

    public static boolean ok() {
        return nativeOk();
    }

    public static Node createNode(String nodeName) {
        long nodeHandle = nativeCreateNodeHandle(nodeName);
        Node node = new Node(nodeHandle);
        return node;
    }

    public static void spinOnce(Node node) {
        long waitSetHandle = nativeGetZeroInitializedWaitSet();

         nativeWaitSetInit(waitSetHandle, node.getSubscriptions().size(), 0, 0);

         nativeWaitSetClearSubscriptions(waitSetHandle);

         for(Subscription subscription : node.getSubscriptions()) {
             nativeWaitSetAddSubscription(waitSetHandle, subscription.getSubscriptionHandle());
         }

         nativeWait(waitSetHandle);

         for(Subscription subscription : node.getSubscriptions()) {
             Object msg = nativeTake(subscription.getSubscriptionHandle(), subscription.getMsgType());
             if (msg != null) {
                 subscription.getCallback().accept(msg);
             }
         }
    }

    private static native void nativeShutdown();

    public static void shutdown() {
        nativeShutdown();
    }

    private static native long nativeGetZeroInitializedWaitSet();

    private static native void nativeWaitSetInit(long waitSetHandle, int numberOfSubscriptions, int numberOfGuardConditions, int numberOfTimers);

    private static native void nativeWaitSetClearSubscriptions(long waitSetHandle);

    private static native void nativeWaitSetAddSubscription(long waitSetHandle, long subscriptionHandle);

    private static native void nativeWait(long waitSetHandle);

    private static native Object nativeTake(long SubscriptionHandle, Class msgType);
}

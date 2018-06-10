/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

import java.lang.management.ManagementFactory;
import java.util.concurrent.ConcurrentMap;
import java.util.concurrent.ConcurrentSkipListMap;

import org.ros2.rcljava.exception.ImplementationAlreadyImportedException;
import org.ros2.rcljava.exception.NoImplementationAvailableException;
import org.ros2.rcljava.exception.NotInitializedException;
import org.ros2.rcljava.executor.DefaultThreadedExecutor;
import org.ros2.rcljava.internal.NativeUtils;
import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.NativeNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.qos.QoSProfile;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * ROS2 java client wrapper.
 *
 * <p>JNI call of ROS2 c client.</p>
 *
 */
public final class RCLJava {

    private static final Logger logger = LoggerFactory.getLogger(RCLJava.class);

    private static final String DISPLAY_SEPARATOR =
            "===============================================================";

    /**
     * The identifier of the currently active RMW implementation.
     */
    private static String rmwImplementation;

    /**
     * Flag to indicate if RCLJava has been fully initialized, with a valid RMW
     *   implementation.
     */
    private static volatile boolean initialized = false;

    private static String[] arguments;

    /**
     * A mapping between RMW implementations and their type supports.
     */
    private static final ConcurrentMap<String, String> RMW_TO_TYPESUPPORT = new ConcurrentSkipListMap<String, String>() {
        /** Serial Id */
        private static final long serialVersionUID = 1L;

        {
            put("rmw_fastrtps_cpp",         "rosidl_typesupport_introspection_c");

            // DISABLE multi-rmw.
//            put("rmw_opensplice_cpp",       "rosidl_typesupport_opensplice_c");
//            put("rmw_connext_cpp",          "rosidl_typesupport_connext_c");
//            put("rmw_connext_dynamic_cpp",  "rosidl_typesupport_introspection_c");
        }
    };

    // Natives definitions
    //################################################################################################################//

    // rcl.h
    /**
     * Initialize the underlying rcl layer.
     */
    private static native void nativeRCLJavaInit(String... args);

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

    // Wait.h
    private static native long nativeConvertQoSProfileToHandle(
            int history, int depth, int reliability, int durability, boolean avoidRos);
    private static native void nativeDisposeQoSProfile(
            long qosProfileHandle);

    /** Release all ressources at shutdown. */
    static {
        Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
                RCLJava.shutdownHook();
            }
        });
    }

    /**
     * Private constructor so this cannot be instantiated.
     */
    private RCLJava() { super(); }

    private static String getRmwImplementationSuffix(final String rmwImplementation) {
        String result = "__" + rmwImplementation;

        if ("__rmw_fastrtps_cpp".equals(result)) {
            result = "";
        }

        return result;
    }

    private static void displayContext() {
        RCLJava.logger.debug(DISPLAY_SEPARATOR);
        // https://docs.oracle.com/javase/7/docs/api/java/lang/System.html#getProperties
        // http://lopica.sourceforge.net/os.html
        final String libpath = System.getProperty("java.library.path");
        final String arch    = System.getProperty("os.arch");
        final String os      = System.getProperty("os.name");
        final String osVer   = System.getProperty("os.version");

        final String pidAndHost = ManagementFactory.getRuntimeMXBean().getName();
        final String pid = pidAndHost.substring(0, pidAndHost.indexOf('@'));
        // For JAVA9 : long pid = ProcessHandle.current().getPid();

        final String user = System.getProperty("user.name");
        final String pathSeparator = System.getProperty("path.separator");

        // Java Value
        RCLJava.logger.debug(String.format("Process ID : %s", pid));
        RCLJava.logger.debug(String.format("Process User : %s", user));

        RCLJava.logger.debug(String.format("Java Home : %s", System.getProperty("java.home")));
        RCLJava.logger.debug(String.format("Java JVM : %s %s %s",
                System.getProperty("java.vm.vendor"),
                System.getProperty("java.vm.name"),
                System.getProperty("java.vm.version")));
        RCLJava.logger.debug(String.format("Java JVM %s : %s",
                System.getProperty("java.vm.specification.name"),
                System.getProperty("java.vm.specification.version")));
        RCLJava.logger.debug(String.format("Java JRE %s : %s",
                System.getProperty("java.specification.name"),
                System.getProperty("java.specification.version")));

        // Native Value
        RCLJava.logger.debug(String.format("Native Library OS : %s %s", os, osVer));
        RCLJava.logger.debug(String.format("Native Library Archi : %s", arch));
        RCLJava.logger.debug(String.format("Native Library path : %n\t%s", libpath.replace(pathSeparator, System.lineSeparator() + "\t")));
        RCLJava.logger.debug(DISPLAY_SEPARATOR);
    }

    private static void displayReport() {
        RCLJava.logger.debug(DISPLAY_SEPARATOR);

        // List loaded libraries.
        final String[] list = NativeUtils.getLoadedLibraries(RCLJava.class.getClassLoader());
        final StringBuilder msgLog = new StringBuilder();
        for (final String key : list) {
            msgLog.append(key);
            msgLog.append(System.lineSeparator());
        }
        RCLJava.logger.debug(String.format("Native libraries Loaded: %n%s", msgLog.toString()));
        RCLJava.logger.debug(DISPLAY_SEPARATOR);
    }

    /**
     * Initialize the RCLJava API. If successful, a valid RMW implementation will
     *   be loaded and accessible, enabling the creating of ROS2 entities
     *   (@{link Node}s, @{link Publisher}s and @{link Subscription}s.
     * @param args CLI arguments.
     */
    public static void rclJavaInit(final String... args) {
        synchronized (RCLJava.class) {
            if (!RCLJava.initialized) {
                if (args != null) {
                    RCLJava.arguments = args;

                    for (final String arg : RCLJava.arguments) {
                        if (arg.contains("=")) {
                            final String[] keyVal = arg.split("=");
                            RCLJava.logger.debug("Args : " + keyVal[0] + "\t : " + keyVal[1]);
                        } else {
                            RCLJava.logger.debug("Args : " + arg);
                        }
                    }
                }

                // Auto-detect RMW implementation.
                if (RCLJava.rmwImplementation == null) {
                    RCLJava.displayContext();
                    RCLJava.autoLoadRmw();
                }

                // No RMW implementation founded !
                if (RCLJava.rmwImplementation == null) {
                    RCLJava.logger.error("No RMW implementation found...");
                    System.exit(1);
                } else

                // RMW implementation founded.
                {
                    RCLJava.logger.debug("Initialize rclJava with " + RCLJava.rmwImplementation);
                    RCLJava.nativeRCLJavaInit(RCLJava.arguments);
                    RCLJava.initialized = true;
                }
            } else {
                final NotInitializedException ex = new NotInitializedException("Cannot intialized twice !");
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

    /**
     * Create a @{link Node}.
     *
     * @param defaultName Name of the node.
     * @return A @{link Node} that represents the underlying ROS2 node
     *     structure.
     */
    public static Node createNode(final String defaultName) {
        RCLJava.logger.debug("Initialize Node stack...");

        return RCLJava.createNode(null, defaultName);
    }

    /**
     * Create a @{link Node}.
     * Take the value from argument app, if you pass null parameter on defaultName.
     *
     * @param namespace Name Space.
     * @param defaultName The name that will identify this node in a ROS2 graph.
     * @return A @{link Node} that represents the underlying ROS2 node
     *     structure.
     */
    public static Node createNode(final String namespace, final String defaultName) {
        return new NativeNode(namespace, defaultName, RCLJava.arguments);
    }

    /**
     * Wait for once loop.
     *
     * @param node Node to spin once only.
     */
    public static void spinOnce(final Node node) {
        RCLJava.lockAndCheckInitialized();

        DefaultThreadedExecutor.getInstance().spinNodeOnce(node, -1);
    }

    /**
     * Helper Spin.
     * @param node Node to spin.
     */
    public static void spin(final Node node) {
        RCLJava.lockAndCheckInitialized();

        DefaultThreadedExecutor.getInstance().spinNodeSome(node);
    }

    /**
     * Return true if rcl is currently initialized, otherwise false.
     *
     * @return true if RCLJava hasn't been shut down, false otherwise.
     */
    public static boolean ok() {
        RCLJava.lockAndCheckInitialized();

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
        RCLJava.shutdown(false);
    }

    private static void shutdown(final boolean force) {
        RCLJava.logger.debug("Shutdown...");

        if (!force) {
            RCLJava.lockAndCheckInitialized();
        }

        if (DefaultThreadedExecutor.hasInstance()) {
            DefaultThreadedExecutor.dispose();
        }

        synchronized (RCLJava.class) {
            if (RCLJava.isInitialized()) {
                RCLJava.nativeShutdown();
                RCLJava.initialized = false;
            }
        }
    }

    /**
     * @return The identifier of the currently active RMW implementation.
     */
    public static String getRMWIdentifier() {
        RCLJava.lockAndCheckInitialized();

        return RCLJava.nativeGetRMWIdentifier();
    }

    /**
     * Get identifier of the ROS2 middle-ware use.
     *
     * <p>TODO rename to list of RMW available.</p>
     *
     * @return Identifier string of ROS2 middle-ware.
     */
    public static String getTypesupportIdentifier() {
        return RMW_TO_TYPESUPPORT.get(RCLJava.getRMWIdentifier());
    }

    /**
     * Switch of ROS2 middle-ware implementation
     *
     * <p>TODO need to check implementation available.</p>
     *
     * @param rmwImplementation
     * @throws NoImplementationAvailableException
     */
    @SuppressWarnings("PMD.AvoidUsingNativeCode")
    public static void setRMWImplementation(final String rmwImplementation)
            throws NoImplementationAvailableException, ImplementationAlreadyImportedException {

        synchronized(RCLJava.class) {
            if (rmwImplementation != null && !rmwImplementation.isEmpty()) {
                if (!rmwImplementation.equals(RCLJava.rmwImplementation)) {
                    final String file = "rcljava"+ RCLJava.getRmwImplementationSuffix(rmwImplementation);
                    RCLJava.logger.debug("Load native RMW file : " + System.mapLibraryName(file));

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
     * Load Native ROS library
     * <i>load from java.library.path .</i>
     * @param name Name of the library.
     */
    @SuppressWarnings("PMD.AvoidUsingNativeCode")
    public static void loadLibrary(final String name) {
        RCLJava.logger.debug("Load native file :" + System.mapLibraryName(name));
        RCLJava.lockAndCheckInitialized();

        try {
            System.loadLibrary(name);  //__" + RCLJava.getRMWIdentifier());
        } catch (UnsatisfiedLinkError e) {
            RCLJava.logger.error("Native code library failed to load.", e);
        }
    }

    /**
     * Load RMW.
     */
    private static void autoLoadRmw() {
        for (final ConcurrentMap.Entry<String, String> entry : RMW_TO_TYPESUPPORT.entrySet()) {
            try {
                RCLJava.logger.debug("Try to load native " + entry.getKey() + "...");
                RCLJava.setRMWImplementation(entry.getKey());
                RCLJava.logger.debug(entry.getKey() + " loaded !");
                break;
            } catch (NoImplementationAvailableException e) {
                RCLJava.logger.error(entry.getKey() + " not available ! (" + e.getMessage() + ")");
            } catch (ImplementationAlreadyImportedException e) {
                RCLJava.logger.error(e.getMessage());
            }
        }
    }

    protected static void shutdownHook() {
        RCLJava.logger.debug("Final Shutdown...");

        RCLJava.shutdown(true);
        GraphName.dispose();

        RCLJava.displayReport();
    }

    private static void lockAndCheckInitialized() {
        synchronized (RCLJava.class) {
            if (!RCLJava.isInitialized()) {
                throw new NotInitializedException();
            }
        }
    }

    /**
     * Convert Java QOS to JNI.
     * @param qosProfile
     * @return handler id.
     */
    public static long convertQoSProfileToHandle(final QoSProfile qosProfile) {
        final int history = qosProfile.getHistory().getValue();
        final int depth = qosProfile.getDepth();
        final int reliability = qosProfile.getReliability().getValue();
        final int durability = qosProfile.getDurability().getValue();
        final boolean avoidRos = qosProfile.getAvoidRosNamespaceConventions();

        // RCLJava.logger.debug("Convert QosProfile...");
        return RCLJava.nativeConvertQoSProfileToHandle(history, depth, reliability, durability, avoidRos);
    }

    /**
     * Dispose JNI QosProfile.
     * @param qosProfileHandle identifier
     */
    public static void disposeQoSProfile(final long qosProfileHandle) {
        RCLJava.nativeDisposeQoSProfile(qosProfileHandle);
    }

}

/* Copyright 2017-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.executor;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.NativeNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.NativeClient;
import org.ros2.rcljava.node.service.NativeService;
import org.ros2.rcljava.node.service.RMWRequestId;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.topic.NativeSubscription;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.time.NativeWallTimer;
import org.ros2.rcljava.time.WallTimer;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class serves as a bridge between ROS2's rcl_waitset_t and RCLJava.
 */
public class NativeExecutor {

    private static final Logger logger = LoggerFactory.getLogger(NativeExecutor.class);

    private final BaseThreadedExecutor executor;
    private final MemoryStrategy memoryStrategy;

    public NativeExecutor(final BaseThreadedExecutor executor) {
        super();
        NativeExecutor.logger.debug("Create Native Executor.");

        this.executor = executor;
        this.memoryStrategy = new MemoryStrategy();
    }

    public void getNextTimer(final AnyExecutable anyExecutable) {
        for (final Node node : this.executor.nodes) {
            for (final WallTimer timer : node.getWallTimers()) {
                if (timer.isReady()) {
                    anyExecutable.timer = timer;
                    return;
                }
            }
        }
    }

    /**
     *
     * @param timeout in ms.
     */
    private void waitForWork(final long timeout) {
        this.memoryStrategy.clearHandles();
        this.memoryStrategy.collectEntities(this.executor.nodes);

     // Initialize Counter.
        int subscriptionsSize = 0;
        int timersSize = 0;
        int clientsSize = 0;
        int servicesSize = 0;

        // Count topics.
        for (final Node node : this.executor.nodes) {
            subscriptionsSize   += node.getSubscriptions().size();
            timersSize          += node.getWallTimers().size();
            clientsSize         += node.getClients().size();
            servicesSize        += node.getServices().size();
        }

        // If any topics exist.
        if (subscriptionsSize > 0 || timersSize > 0 || clientsSize > 0 || servicesSize > 0) {
            final long waitSetHandle = NativeExecutor.nativeGetZeroInitializedWaitSet();

            NativeExecutor.nativeWaitSetInit(
                    waitSetHandle,
                    subscriptionsSize,
                    0,
                    timersSize,
                    clientsSize,
                    servicesSize);

            // Clean Waitset components.
            NativeExecutor.nativeWaitSetClearSubscriptions(waitSetHandle);
            NativeExecutor.nativeWaitSetClearTimers(waitSetHandle);
            NativeExecutor.nativeWaitSetClearServices(waitSetHandle);
            NativeExecutor.nativeWaitSetClearClients(waitSetHandle);


            for (final Node node : this.executor.nodes) {
                final NativeNode nativeNode = (NativeNode) node;

                // Subscribe waiset components.
                for (final NativeSubscription<?> subscription : nativeNode.getNativeSubscriptions()) {
                    NativeExecutor.nativeWaitSetAddSubscription(waitSetHandle, subscription.getSubscriptionHandle());
                }

                for (final NativeWallTimer timer : nativeNode.getNativeWallTimers()) {
                    NativeExecutor.nativeWaitSetAddTimer(waitSetHandle, timer.getHandle());
                }

                for (final NativeService<?> service : nativeNode.getNativeServices()) {
                    NativeExecutor.nativeWaitSetAddService(waitSetHandle, service.getServiceHandle());
                }

                for (final NativeClient<?> client : nativeNode.getNativeClients()) {
                    NativeExecutor.nativeWaitSetAddClient(waitSetHandle, client.getClientHandle());
                }
            }

            // Wait...
            NativeExecutor.nativeWait(waitSetHandle, timeout);
            NativeExecutor.nativeWaitSetFini(waitSetHandle);
        }
    }

    /**
     *
     * @return
     */
    public AnyExecutable getNextReadyExecutable() {
        final AnyExecutable anyExecutable = this.memoryStrategy.instantiateNextExecutable();

        // Check the timers to see if there are any that are ready, if so return
        this.getNextTimer(anyExecutable);
        if (anyExecutable.timer != null) {
            return anyExecutable;
        }

        // Check the subscriptions to see if there are any that are ready
        this.memoryStrategy.getNextSubscription(anyExecutable, this.executor.nodes);
        if (anyExecutable.subscription != null) {
            return anyExecutable;
        }

        // Check the services to see if there are any that are ready
        this.memoryStrategy.getNextService(anyExecutable, this.executor.nodes);
        if (anyExecutable.service != null) {
            return anyExecutable;
        }
        // Check the clients to see if there are any that are ready
        this.memoryStrategy.getNextClient(anyExecutable, this.executor.nodes);
        if (anyExecutable.client != null) {
            return anyExecutable;
        }

        // If there is no ready executable, return a null.
        return null;
    }

    public AnyExecutable getNextExecutable() {
        return this.getNextExecutable(0);
    }

    /**
     *
     * @param timeout
     * @return
     */
    public synchronized AnyExecutable getNextExecutable(final long timeout) {
        AnyExecutable anyExecutable = this.getNextReadyExecutable();

        if (anyExecutable == null) {
            this.waitForWork(timeout);
            anyExecutable = this.getNextReadyExecutable();
        }

        return anyExecutable;
    }

    /**
     * Execute any type from Memory.
     * @param anyExecutable
     */
    public static void executeAnyExecutable(final AnyExecutable anyExecutable) {

        if (anyExecutable == null) {
            return;
        }

        if (anyExecutable.timer != null) {
            NativeExecutor.executeTimer(anyExecutable.timer);
        }

        if (anyExecutable.subscription != null) {
            NativeExecutor.executeSubscription(anyExecutable.subscription);
        }

        if (anyExecutable.service != null) {
            NativeExecutor.executeService(anyExecutable.service);
        }

        if (anyExecutable.client != null) {
            NativeExecutor.executeClient(anyExecutable.client);
        }
    }

    /**
     * Execute Timer from Memory.
     * @param timer to execute.
     */
    private static void executeTimer(final WallTimer timer) {
        timer.callTimer();
        timer.getCallback().tick();
    }

    /**
     * Execute Subscription from Memory.
     * @param subscription to execute.
     */
    @SuppressWarnings({ "unchecked", "rawtypes" })
    private static void executeSubscription(final Subscription subscription) {
        final NativeSubscription<? extends Message> nativeSubscription = (NativeSubscription<?> ) subscription;
        final Message message = NativeExecutor.nativeTake(nativeSubscription.getSubscriptionHandle(), nativeSubscription.getMessageType());
        if (message != null) {
            subscription.getCallback().dispatch(message);
        }
    }

    /**
     * Execute Service from Memory.
     * @param service to execute.
     */
    @SuppressWarnings({ "unchecked", "rawtypes" })
    private static void executeService(final Service service) {
        final NativeService<?> nativeService = (NativeService<?>) service;
        final Class<?> requestType = service.getRequestType();
        final Class<?> responseType = service.getResponseType();

        Message requestMessage  = null;
        Message responseMessage = null;

        try {
            requestMessage  = (Message) requestType.newInstance();
            responseMessage = (Message) responseType.newInstance();
        } catch (InstantiationException ie) {
            ie.printStackTrace();
        } catch (IllegalAccessException iae) {
            iae.printStackTrace();
        }

        if (requestMessage != null && responseMessage != null) {
            final RMWRequestId rmwRequestId = NativeExecutor.nativeTakeRequest(
                    nativeService.getServiceHandle(),
                    nativeService.getRequest().getFromJavaConverterHandle(),
                    nativeService.getRequest().getToJavaConverterHandle(),
                    requestMessage);

            if (rmwRequestId != null) {
                service.getCallback().dispatch(rmwRequestId, requestMessage, responseMessage);
                NativeExecutor.nativeSendServiceResponse(
                        nativeService.getServiceHandle(),
                        rmwRequestId,
                        nativeService.getResponse().getFromJavaConverterHandle(),
                        nativeService.getResponse().getToJavaConverterHandle(),
                        responseMessage);
            }
        }
    }

    /**
     * Execute Client from Memory.
     * @param client to execute.
     */
    @SuppressWarnings("rawtypes")
    private static void executeClient(final Client client) {

    }

    public static native long nativeGetZeroInitializedWaitSet();
    public static native void nativeWaitSetInit(
            long waitSetHandle,
            int numberOfSubscriptions,
            int numberOfGuardConditions,
            int numberOfTimers,
            int numberOfClients,
            int numberOfServices);
    public static native void nativeWaitSetClearSubscriptions(long waitSetHandle);
    public static native void nativeWaitSetAddSubscription(long waitSetHandle, long subscriptionHandle);
    public static native void nativeWaitSetClearServices(long waitSetHandle);
    public static native void nativeWaitSetAddService(long waitSetHandle, long serviceHandle);
    public static native void nativeWaitSetClearTimers(long waitSetHandle);
    public static native void nativeWaitSetAddTimer(long waitSetHandle, long timerHandle);
    public static native void nativeWaitSetClearClients(long waitSetHandle);
    public static native void nativeWaitSetAddClient(long waitSetHandle, long clientHandle);
    public static native void nativeWait(long waitSetHandle, long timeout);
    public static native void nativeWaitSetFini(long waitSetHandle);

    public static native Message nativeTake(long SubscriptionHandle, Class<?> msgType);
    public static native RMWRequestId nativeTakeRequest(
            long serviceHandle,
            long requestFromJavaConverterHandle,
            long requestToJavaConverterHandle,
            Object requestMessage);
    public static native void nativeSendServiceResponse(
            long serviceHandle,
            Object header,
            long responseFromJavaConverterHandle,
            long responseToJavaConverterHandle,
            Object responseMessage);
    public static native Object nativeTakeResponse(
            long clientHandle,
            long responseFromJavaConverterHandle,
            long responseToJavaConverterHandle,
            Object responseMessage);
}

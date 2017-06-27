/* Copyright 2017 Mickael Gaillard <mick.gaillard@gmail.com>
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

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.NativeClient;
import org.ros2.rcljava.node.service.RMWRequestId;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.topic.NativeSubscription;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.time.WallTimer;


/**
 * This class serves as a bridge between ROS2's rcl_waitset_t and RCLJava.
 */
public class NativeExecutor {

    private BaseThreadedExecutor executor;
    private MemoryStrategy memoryStrategy;

    public NativeExecutor(BaseThreadedExecutor executor) {
        this.executor = executor;
        this.memoryStrategy = new MemoryStrategy();
    }

    public void getNextTimer(AnyExecutable anyExecutable) {
        for (Node node : this.executor.nodes) {
            for (WallTimer timer : node.getWallTimers()) {
                if (timer.isReady()) {
                    anyExecutable.timer = timer;
                    return;
                }
            }
        }
    }

    /**
     *
     */
    @SuppressWarnings({ "resource" })
    private void waitForWork(long timeout) {

        memoryStrategy.clearHandles();
//        boolean hasInvalidNodes =
        memoryStrategy.collectEntities(this.executor.nodes);

        int subscriptionsSize = 0;
        int timersSize = 0;
        int clientsSize = 0;
        int servicesSize = 0;

        for (Node node : this.executor.nodes) {
            subscriptionsSize   += node.getSubscriptions().size();
            timersSize          += node.getWallTimers().size();
            clientsSize         += node.getClients().size();
            servicesSize        += node.getServices().size();
        }

        if (subscriptionsSize > 0 || timersSize > 0 || clientsSize > 0 || servicesSize > 0) {
            long waitSetHandle = RCLJava.nativeGetZeroInitializedWaitSet();

            RCLJava.nativeWaitSetInit(
                    waitSetHandle,
                    subscriptionsSize,
                    0,
                    timersSize,
                    clientsSize,
                    servicesSize);

            // Clean Waitset components.
            RCLJava.nativeWaitSetClearSubscriptions(waitSetHandle);
            RCLJava.nativeWaitSetClearTimers(waitSetHandle);
            RCLJava.nativeWaitSetClearServices(waitSetHandle);
            RCLJava.nativeWaitSetClearClients(waitSetHandle);


            for (Node node : this.executor.nodes) {
                // Subscribe waiset components.
                for (Subscription<?> subscription : node.getSubscriptions()) {
                    NativeSubscription<?> nativeSubscription = (NativeSubscription<?>) subscription;
                    RCLJava.nativeWaitSetAddSubscription(waitSetHandle, nativeSubscription.getSubscriptionHandle());
                }

                for (WallTimer timer : node.getWallTimers()) {
                    RCLJava.nativeWaitSetAddTimer(waitSetHandle, timer.getHandle());
                }

                for (Service<?> service : node.getServices()) {

                    RCLJava.nativeWaitSetAddService(waitSetHandle, service.getServiceHandle());
                }

                for (Client<?> client : node.getClients()) {
                    NativeClient<?> nativeClient = (NativeClient<?>) client;
                    RCLJava.nativeWaitSetAddClient(waitSetHandle, nativeClient.getClientHandle());
                }
            }

            // Wait...
            RCLJava.nativeWait(waitSetHandle);
            RCLJava.nativeWaitSetFini(waitSetHandle);
        }
    }

    /**
     *
     * @return
     */
    public AnyExecutable getNextReadyExecutable() {
        AnyExecutable anyExecutable = this.memoryStrategy.instantiateNextExecutable();

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
    public synchronized AnyExecutable getNextExecutable(long timeout) {
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
        NativeSubscription<? extends Message> nativeSubscription = (NativeSubscription<?> ) subscription;
        Message message = RCLJava.nativeTake(nativeSubscription.getSubscriptionHandle(), nativeSubscription.getMessageType());
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
        Class<?> requestType = service.getRequestType();
        Class<?> responseType = service.getResponseType();

        Message requestMessage = null;
        Message responseMessage = null;

        try {
            requestMessage = (Message) requestType.newInstance();
            responseMessage = (Message) responseType.newInstance();
        } catch (InstantiationException ie) {
            ie.printStackTrace();
        } catch (IllegalAccessException iae) {
            iae.printStackTrace();
        }

        if (requestMessage != null && responseMessage != null) {
            long requestFromJavaConverterHandle = service.getRequestFromJavaConverterHandle();
            long requestToJavaConverterHandle = service.getRequestToJavaConverterHandle();
            long responseFromJavaConverterHandle = service.getResponseFromJavaConverterHandle();
            long responseToJavaConverterHandle = service.getResponseToJavaConverterHandle();

            RMWRequestId rmwRequestId = (RMWRequestId) RCLJava.nativeTakeRequest(
                    service.getServiceHandle(),
                    requestFromJavaConverterHandle,
                    requestToJavaConverterHandle,
                    requestMessage);

            if (rmwRequestId != null) {
                service.getCallback().dispatch(rmwRequestId, requestMessage, responseMessage);
                RCLJava.nativeSendServiceResponse(
                        service.getServiceHandle(),
                        rmwRequestId,
                        responseFromJavaConverterHandle,
                        responseToJavaConverterHandle,
                        responseMessage);
            }
        }
    }

    @SuppressWarnings("rawtypes")
    private static void executeClient(Client client) {

    }
}

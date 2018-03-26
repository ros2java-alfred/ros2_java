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

package org.ros2.rcljava.node.service;

import java.lang.ref.WeakReference;
import java.util.concurrent.Future;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.Node;

/**
 * Service Client.
 *
 * @param <T> Service Type.
 */
public class NativeClient<T extends MessageService> extends BaseClient<T> {

    // Loading JNI library.
    static {
        RCLJava.loadLibrary("rcljava_node_service_NativeClient");
    }

    /** Client Handler. */
    private final long clientHandle;

    private final long requestFromJavaConverterHandle;
    private final long requestToJavaConverterHandle;

    private final long responseFromJavaConverterHandle;
    private final long responseToJavaConverterHandle;

    private static native void nativeSendClientRequest(
            long clientHandle,
            long sequenceNumber,
            long requestFromJavaConverterHandle,
            long requestToJavaConverterHandle,
            Object requestMessage);

    /**
     *
     * @param nodeReference
     * @param clientHandle
     * @param serviceType
     * @param serviceName
     * @param requestType
     * @param responseType
     * @param requestFromJavaConverterHandle
     * @param requestToJavaConverterHandle
     * @param responseFromJavaConverterHandle
     * @param responseToJavaConverterHandle
     */
    public NativeClient(
            final WeakReference<Node> nodeReference,
            final long clientHandle,
            final Class<T> serviceType,
            final String serviceName,
            final Class<? extends Message> requestType,
            final Class<? extends Message> responseType,
            final long requestFromJavaConverterHandle,
            final long requestToJavaConverterHandle,
            final long responseFromJavaConverterHandle,
            final long responseToJavaConverterHandle) {
        super(nodeReference, serviceType, serviceName, requestType, responseType);

        if (clientHandle == 0) { throw new RuntimeException("Need to provide active node with handle object"); }
        this.clientHandle = clientHandle;

        this.requestFromJavaConverterHandle = requestFromJavaConverterHandle;
        this.requestToJavaConverterHandle = requestToJavaConverterHandle;
        this.responseFromJavaConverterHandle = responseFromJavaConverterHandle;
        this.responseToJavaConverterHandle = responseToJavaConverterHandle;
    }

    @Override
    public final <U extends Message, V extends Message> Future<V> sendRequest(final U request) {
        synchronized(this.getPendingRequests()) {
              this.incrementSequenceNumber();

              NativeClient.nativeSendClientRequest(
                      this.clientHandle,
                      this.getSequenceNumber(),
                      this.requestFromJavaConverterHandle,
                      this.requestToJavaConverterHandle,
                      request);

              final RCLFuture<V> future = new RCLFuture<V>(this.getNode());
              getPendingRequests().put(this.getSequenceNumber(), future);
              return future;
            }
    }

    public final long getClientHandle() {
        return this.clientHandle;
    }

    public final long getRequestFromJavaConverterHandle() {
        return this.requestFromJavaConverterHandle;
    }

    public final long getRequestToJavaConverterHandle() {
        return this.requestToJavaConverterHandle;
    }

    public final long getResponseFromJavaConverterHandle() {
        return this.responseFromJavaConverterHandle;
    }

    public final long getResponseToJavaConverterHandle() {
        return this.responseToJavaConverterHandle;
    }
}

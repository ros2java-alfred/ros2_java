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
import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.NativeNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.qos.QoSProfile;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class is Native(rcl) Service Client of RCLJava.
 *
 * @param <T> Service Type.
 */
public class NativeClient<T extends MessageService> extends BaseClient<T> {

    private static final Logger logger = LoggerFactory.getLogger(NativeClient.class);

    /** Client Handler. */
    private final long clientHandle;

    private final NativeServiceType<T> request;
    private final NativeServiceType<T> response;

    private static native <T> long nativeCreateClientHandle(
            long nodeHandle, Class<T> cls, String serviceName, long qosProfileHandle);

    private static native void nativeDispose(long nodeHandle, long clientHandle);

    private static native long nativeSendClientRequest(
            long clientHandle,
            long requestFromJavaConverterHandle,
            long requestToJavaConverterHandle,
            Object requestMessage);

    /**
     * Constructor.
     *
     * @param nodeReference
     * @param serviceType
     * @param serviceName
     * @param request
     * @param response
     * @param qosProfile
     */
    public NativeClient(
            final NativeNode nodeReference,
            final Class<T> serviceType,
            final String serviceName,
            final NativeServiceType<T> request,
            final NativeServiceType<T> response,
            final QoSProfile qosProfile) {
        super(nodeReference, serviceType, serviceName, request.getType(), response.getType(), qosProfile);

        final String fqnService =  GraphName.getFullName(nodeReference, serviceName, null);
        if (!GraphName.isValidTopic(fqnService)) { throw new RuntimeException("Invalid topic name."); }

        final long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
        this.clientHandle = NativeClient.nativeCreateClientHandle(
                nodeReference.getNodeHandle(),
                serviceType,
                fqnService,
                qosProfileHandle);
        RCLJava.disposeQoSProfile(qosProfileHandle);
        if (this.clientHandle == 0) { throw new RuntimeException("Need to provide active node with handle object"); }

        this.request = request;
        this.response = response;

        NativeClient.logger.debug(
                String.format("Created Native Service Client : %s  [0x%x] (request : [0x%x]> <[0x%x]) (response : [0x%x]> <[0x%x])",
                        this.getServiceName(),
                        this.clientHandle,
                        this.request.getFromJavaConverterHandle(),
                        this.request.getToJavaConverterHandle(),
                        this.response.getFromJavaConverterHandle(),
                        this.response.getToJavaConverterHandle()));
    }

    @Override
    public void dispose() {
        super.dispose();

        NativeClient.logger.debug(
                String.format("Destroy Native Service Client : %s  [0x%x] (request : [0x%x]> <[0x%x]) (response : [0x%x]> <[0x%x])",
                        this.getServiceName(),
                        this.clientHandle,
                        this.request.getFromJavaConverterHandle(),
                        this.request.getToJavaConverterHandle(),
                        this.response.getFromJavaConverterHandle(),
                        this.response.getToJavaConverterHandle()));

        NativeClient.nativeDispose(this.getNode().getNodeHandle(), this.clientHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getNode()
     */
    @Override
    public NativeNode getNode() {
        return (NativeNode) super.getNode();
    }

    @Override
    public <U extends Message, V extends Message> Future<V> sendRequest(final U request) {
        final RCLFuture<V> future;

        synchronized(this.getPendingRequests()) {
              final Long sequenceNumber = NativeClient.nativeSendClientRequest(
                      this.clientHandle,
                      this.request.getFromJavaConverterHandle(),
                      this.request.getToJavaConverterHandle(),
                      request);

              future = new RCLFuture<V>(new WeakReference<Node>(this.getNode()));
              this.getPendingRequests().put(sequenceNumber, future);
        }
        return future;
    }

    public long getClientHandle() {
        return this.clientHandle;
    }

    public NativeServiceType<T> getRequest() {
        return this.request;
    }

    public NativeServiceType<T> getResponse() {
        return this.response;
    }
}

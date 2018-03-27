/* Copyright 2018 Mickael Gaillard <mick.gaillard@gmail.com>
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
import java.util.HashMap;
import java.util.Map;

import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.Node;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class is Service Client of RCLJava.
 *
 * @param <T> Service Type.
 */
public abstract class BaseClient<T extends MessageService> implements Client<T> {

    private static final Logger logger = LoggerFactory.getLogger(BaseClient.class);

    private final WeakReference<Node> nodeReference;

    private final Class<T> serviceType;
    private final String serviceName;
    private final Map<Long, RCLFuture<?>> pendingRequests;

    private final Class<? extends Message> requestType;
    private final Class<? extends Message> responseType;

    private long sequenceNumber;

    /**
     *
     * @param nodeReference
     * @param serviceType
     * @param serviceName
     * @param requestType
     * @param responseType
     */
    public BaseClient(
            final WeakReference<Node> nodeReference,
            final Class<T> serviceType,
            final String serviceName,
            final Class<? extends Message> requestType,
            final Class<? extends Message> responseType) {

        if (nodeReference == null) { throw new RuntimeException("Need to provide active node with handle object"); }
        this.nodeReference = nodeReference;

        this.serviceType = serviceType;
        this.serviceName = serviceName;
        this.requestType = requestType;
        this.responseType = responseType;

        this.pendingRequests = new HashMap<Long, RCLFuture<?>>();

        BaseClient.logger.debug("Create Client of topic : " + this.serviceName);
        this.nodeReference.get().getClients().add(this);
    }

    @Override
    public void close() throws Exception {
        this.dispose();
    }

    @Override
    public void dispose() {
        BaseClient.logger.debug("Destroy Client of topic : " + this.serviceName);

        if (this.nodeReference.get().getClients().contains(this)) {
            this.nodeReference.get().getClients().remove(this);
        }
    }

    @Override
    public Class<? extends Message> getRequestType() {
        return this.requestType;
    }

    @Override
    public Class<? extends Message> getResponseType() {
        return this.responseType;
    }

    public String getServiceName() {
        return this.serviceName;
    }

    public Class<T> getServiceType() {
        return this.serviceType;
    }

    public <V extends Message> void handleResponse(final RMWRequestId header,final V response) {
        synchronized(pendingRequests) {
            final long sequenceNumber = header.sequenceNumber;
            @SuppressWarnings("unchecked")
            final RCLFuture<V> future = (RCLFuture<V>) pendingRequests.remove(sequenceNumber);
            future.set(response);
        }
    }

    public boolean waitForService(final int i) {
        //TODO
        throw new NotImplementedException();
//          return false;
    }

    public WeakReference<Node> getNode() {
        return this.nodeReference;
    }

    public long getSequenceNumber() {
        return this.sequenceNumber;
    }

    public void incrementSequenceNumber() {
        this.sequenceNumber++;
    }

    public Map<Long, RCLFuture<?>> getPendingRequests() {
        return this.pendingRequests;
    }

}

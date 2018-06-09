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

import java.util.HashMap;
import java.util.Map;

import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.qos.QoSProfile;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class is Service Client of RCLJava.
 *
 * @param <T> Service Type.
 */
public abstract class BaseClient<T extends MessageService> implements Client<T> {

    private static final Logger logger = LoggerFactory.getLogger(BaseClient.class);

    private final Node ownerNode;

    private final Class<T> serviceType;
    private final String serviceName;
    private final QoSProfile qosProfile;
    private final Map<Long, RCLFuture<?>> pendingRequests;

    private final Class<? extends Message> requestType;
    private final Class<? extends Message> responseType;

    /**
     *
     * @param ownerNode
     * @param serviceType
     * @param serviceName
     * @param requestType
     * @param responseType
     */
    public BaseClient(
            final Node ownerNode,
            final Class<T> serviceType,
            final String serviceName,
            final Class<? extends Message> requestType,
            final Class<? extends Message> responseType,
            final QoSProfile qosProfile) {

        if (ownerNode == null) { throw new RuntimeException("Need to provide active node with handle object"); }
        this.ownerNode = ownerNode;

        this.serviceType = serviceType;
        this.serviceName = serviceName;
        this.qosProfile  = qosProfile;
        this.requestType = requestType;
        this.responseType = responseType;

        this.pendingRequests = new HashMap<Long, RCLFuture<?>>();

        BaseClient.logger.debug("Create Client of topic : " + this.serviceName);
        this.ownerNode.getClients().add(this);
    }

    @Override
    public void close() throws Exception {
        this.dispose();
    }

    @Override
    public void dispose() {
        BaseClient.logger.debug("Destroy Client of topic : " + this.serviceName);

        if (this.ownerNode.getClients().contains(this)) {
            this.ownerNode.getClients().remove(this);
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
            final RCLFuture<V> future = (RCLFuture<V>) this.pendingRequests.remove(sequenceNumber);
            future.set(response);
        }
    }

    public boolean waitForService(final int i) {
        //TODO
        throw new NotImplementedException();
//          return false;
    }

    public Node getNode() {
        return this.ownerNode;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getQosProfile()
     */
//    @Override
    public QoSProfile getQosProfile() {
        return this.qosProfile;
    }

    public Map<Long, RCLFuture<?>> getPendingRequests() {
        return this.pendingRequests;
    }

}

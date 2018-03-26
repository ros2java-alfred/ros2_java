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

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.Node;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @param <T>
 */
public class BaseService<T extends MessageService> implements Service<T> {

    private static final Logger logger = LoggerFactory.getLogger(BaseService.class);

    /** Node owner. */
    private final Node ownerNode;

    private final Class<T> serviceType;

    /** Name of the service */
    private final String serviceName;

    private final ServiceCallback<?, ?> callback;

    private final Class<? extends Message> requestType;
    private final Class<? extends Message> responseType;

    /**
     *
     * @param node
     * @param serviceType
     * @param serviceName
     * @param callback
     * @param requestType
     * @param responseType
     */
    public BaseService(
            final Node node,
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback,
            final Class<? extends Message> requestType,
            final Class<? extends Message> responseType) {

        BaseService.logger.debug("Init Service stack : " + serviceName);

        if (node == null) { throw new RuntimeException("Need to provide active node with handle object"); }
        this.ownerNode = node;

        this.serviceType = serviceType;
        this.serviceName = serviceName;
        this.callback = callback;
        this.requestType = requestType;
        this.responseType = responseType;

        this.ownerNode.getServices().add(this);
    }

    /* (non-Javadoc)
     * @see java.lang.AutoCloseable#close()
     */
    @Override
    public void close() throws Exception {
        this.dispose();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.service.Service#dispose()
     */
    @Override
    public void dispose() {
        BaseService.logger.debug("Destroy Service stack : " + this.serviceName);

        if (this.ownerNode.getServices().contains(this)) {
            this.ownerNode.getServices().remove(this);
        }
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.service.Service#getCallback()
     */
    @Override
    public ServiceCallback<?, ?> getCallback() {
        return this.callback;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.service.Service#getNode()
     */
    @Override
    public Node getNode() {
        return this.ownerNode;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.service.Service#getRequestType()
     */
    @Override
    public Class<? extends Message> getRequestType() {
        return this.requestType;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.service.Service#getResponseType()
     */
    @Override
    public Class<? extends Message> getResponseType() {
        return this.responseType;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.service.Service#getServiceName()
     */
    @Override
    public String getServiceName() {
        return this.serviceName;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.service.Service#getServiceType()
     */
    @Override
    public Class<T> getServiceType() {
        return this.serviceType;
    }

}

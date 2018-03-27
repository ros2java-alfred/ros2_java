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

import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.NativeNode;
import org.ros2.rcljava.node.Node;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class is Native(rcl) Service Server of RCLJava.
 *
 * @param <T> Service Type.
 */
public class NativeService<T extends MessageService> extends BaseService<T> {

    private static final Logger logger = LoggerFactory.getLogger(NativeService.class);

    /** Service Handler. */
    private final long serviceHandle;

    private final NativeServiceType<T> request;
    private final NativeServiceType<T> response;

    /**
     *
     * @param node
     * @param serviceHandle
     * @param serviceType
     * @param serviceName
     * @param callback
     * @param request
     * @param response
     */
    public NativeService(
            final Node node,
            final long serviceHandle,
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback,
            final NativeServiceType<T> request,
            final NativeServiceType<T> response) {
        super(node, serviceType, serviceName, callback, request.getType(), response.getType());

        NativeService.logger.debug("Init Native Service stack : " + serviceName);

        if (serviceHandle == 0) { throw new RuntimeException("Need to provide active service with handle object"); }
        this.serviceHandle = serviceHandle;

        this.request = request;
        this.response = response;
    }

    public void dispose() {
        NativeService.logger.debug("Destroy Service stack : " + this.getServiceName());

        super.dispose();
    }

//    public void sendResponse() {
//
//    }

    @Override
    public NativeNode getNode() {
        return (NativeNode) this.getNode();
    }

    public long getServiceHandle() {
        return this.serviceHandle;
    }

    public NativeServiceType<T> getRequest() {
        return this.request;
    }

    public NativeServiceType<T> getResponse() {
        return this.response;
    }

}

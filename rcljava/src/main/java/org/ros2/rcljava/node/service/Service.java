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
package org.ros2.rcljava.node.service;

import org.ros2.rcljava.RMWRequestId;

/**
 * Service Server.
 *
 * @param <T> Service Type.
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class Service<T> {

    /** Name of the node */
    private final String serviceName;

    /** Node Handler. */
//    private final long nodeHandle;

    /** Service Handler. */
    private final long serviceHandle;

    private final Class<T> serviceType;

    private final TriConsumer<RMWRequestId, ?, ?> callback;

    private final long requestFromJavaConverterHandle;
    private final long requestToJavaConverterHandle;

    private final long responseFromJavaConverterHandle;
    private final long responseToJavaConverterHandle;

    private final Class<?> requestType;
    private final Class<?> responseType;

    /**
     * Constructor.
     *
     * @param nodeHandle
     * @param serviceName
     */
    public Service(
            final long nodeHandle,
            final long serviceHandle,
            final Class<T> serviceType,
            final String serviceName,
            final TriConsumer<RMWRequestId, ?, ?> callback,
            final Class<?> requestType,
            final Class<?> responseType,
            final long requestFromJavaConverterHandle,
            final long requestToJavaConverterHandle,
            final long responseFromJavaConverterHandle,
            final long responseToJavaConverterHandle) {
//        this.nodeHandle = nodeHandle;
        this.serviceHandle = serviceHandle;
        this.serviceType = serviceType;
        this.serviceName = serviceName;
        this.callback = callback;
        this.requestType = requestType;
        this.responseType = responseType;
        this.requestFromJavaConverterHandle = requestFromJavaConverterHandle;
        this.requestToJavaConverterHandle = requestToJavaConverterHandle;
        this.responseFromJavaConverterHandle = responseFromJavaConverterHandle;
        this.responseToJavaConverterHandle = responseToJavaConverterHandle;
    }

    public void dispose() {
        //TODO
    }

    public void sendResponse() {

    }

    public String getServiceName() {
        return this.serviceName;
    }

    public final TriConsumer<RMWRequestId, ?, ?> getCallback() {
        return callback;
    }

    public final Class<T> getServiceType() {
        return serviceType;
    }

    public final long getServiceHandle() {
        return this.serviceHandle;
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

    public final Class<?> getRequestType() {
        return this.requestType;
    }

    public final Class<?> getResponseType() {
        return this.responseType;
    }
}
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

import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.Node;

/**
 * This class is JVM Service Server of RCLJava.
 * <b>Actually not implemented !!!</b>
 *
 * @param <T> Service Type.
 */
public class JavaService<T extends MessageService> extends BaseService<T> {

    /**
     *
     * @param node
     * @param serviceType
     * @param serviceName
     * @param callback
     * @param requestType
     * @param responseType
     */
    public JavaService(
            final Node node,
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback,
            final Class<? extends Message> requestType,
            final Class<? extends Message> responseType) {
        super(node, serviceType, serviceName, callback, requestType, responseType);

        throw new NotImplementedException();
    }

}

/* Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

import java.util.concurrent.Future;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;

/**
 * Interface of Service Client.
 *
 * @param <T> Service Type.
 */
public interface Client<T extends MessageService> extends AutoCloseable {

    /**
     * Safely destroy the underlying ROS2 Client structure.
     */
    void dispose();

    /**
     *
     * @return
     */
    Class<? extends Message> getResponseType();

    /**
     *
     * @param rmwRequestId
     * @param responseMessage
     */
    <U extends Message> void handleResponse(final RMWRequestId rmwRequestId, final U responseMessage);

    /**
     * Query Service.
     *
     * @param <U> Request message Type.
     * @param <V> Responce message Type.
     * @param request Request of the service.
     * @return Futur of Responce.
     */
    <U extends Message, V extends Message> Future<V> sendRequest(final U request);

}

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

package org.ros2.rcljava.node.internal;

import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.qos.QoSProfile;

/**
 * Interface of Node Services stack.
 */
public interface NodeServices {

    /**
     * Create and return a Client.
     *
     * @param <T> Message definition.
     * @param serviceType Message Class
     * @param serviceName The service to subscribe on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Client instance of the service.
     */
    <T extends MessageService> Client<T> createClient(
            final Class<T> serviceType,
            final String serviceName,
            final QoSProfile qos);

    /**
     * Create and return a Client. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param serviceType Message Class
     * @param serviceName The service to subscribe on.
     * @return Client instance of the service.
     */
    <T extends MessageService> Client<T> createClient(
            final Class<T> serviceType,
            final String serviceName);

//    /**
//     *
//     * @param client
//     */
//    <T extends MessageService> void addClient(final Client<T> client);

    /**
     * Create and return a Service.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param serviceName The service for this publisher to publish on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Service instance of the service.
     */
    <T extends MessageService> Service<T> createService(
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback,
            final QoSProfile qos);

    /**
     * Create and return a Service. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param serviceName The service for this publisher to publish on.
     * @param callback The user-defined callback function.
     * @return Service instance of the service.
     */
    <T extends MessageService> Service<T> createService(
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback);

//    /**
//     *
//     * @param service
//     */
//   <T extends MessageService> void addService(final Service<T> service);

}

/* Copyright 2016 Open Source Robotics Foundation, Inc.
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
package org.ros2.rcljava;

import java.util.HashMap;
import java.util.List;

/**
 * ROS2 Client API.
 *
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public interface INode {

    /**
     * Release all ressource.
     */
    void dispose();

    /**
     * Get the name of the node.
     *
     * @return The name of the node.
     */
    String getName();

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topic The topic for this publisher to publish on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Publisher instance of the created publisher.
     */
    <T> Publisher<T> createPublisher(Class<T> message, String topic, QoSProfile qos);

    /**
     * Create and return a Publisher. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topic The topic for this publisher to publish on.
     * @return Publisher instance of the created publisher.
     */
    <T> Publisher<T> createPublisher(Class<T> message, String topic);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topic The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Subscription instance of the created subscription.
     */
    <T> Subscription<T> createSubscription(Class<T> message, String topic, Consumer<T> callback, QoSProfile qos);

    /**
     * Create and return a Subscription. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topic The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @return Subscription instance of the created subscription.
     */
    <T> Subscription<T> createSubscription(Class<T> message, String topic, Consumer<T> callback);

    /**
     * Create and return a Client.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service to subscribe on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Client instance of the service.
     */
    <T> Client<T> createClient(Class<T> message, String service, QoSProfile qos);

    /**
     * Create and return a Client. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service to subscribe on.
     * @return Client instance of the service.
     */
    <T> Client<T> createClient(Class<T> message, String service);

    /**
     * Create and return a Service.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service for this publisher to publish on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Service instance of the service.
     */
    <T> Service<T> createService(Class<T> message, String service, Consumer<T> callback, QoSProfile qos);

    /**
     * Create and return a Service. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service for this publisher to publish on.
     * @param callback The user-defined callback function.
     * @return Service instance of the service.
     */
    <T> Service<T> createService(Class<T> message, String service, Consumer<T> callback);

    List<Object> setParameters(List<Object> parameters);

    List<Object> getParameters(List<String> names);

    Object getParameter(String name);

    HashMap<String, String> getTopicNamesAndTypes();

    int countPublishers(String topic);

    int countSubscribers(String topic);

    /**
     * Return the rcl_node_t node handle (non-const version).
     * @return
     */
    long getRclNodeHandle();

    /**
     * Notify threads waiting on graph changes.
     *
     * Affects threads waiting on the notify guard condition, see:
     * get_notify_guard_condition(), as well as the threads waiting on graph
     * changes using a graph Event, see: wait_for_graph_change().
     *
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     *
     * \throws RCLBaseError (a child of that exception) when an rcl error occurs
     */
    void notifyGraphChange();

    /** Notify any and all blocking node actions that shutdown has occurred. */
    void notifyShutdown();

    /**
     * Return a graph event, which will be set anytime a graph change occurs.
     *
     * The graph Event object is a loan which must be returned.
     * The Event object is scoped and therefore to return the load just let it go
     * out of scope.
     */
    Object getGraphEvent();

    /**
     * Wait for a graph event to occur by waiting on an Event to become set.
     *
     * The given Event must be acquire through the get_graph_event() method.
     *
     * \throws InvalidEventError if the given event is nullptr
     * \throws EventNotRegisteredError if the given event was not acquired with
     *   get_graph_event().
     */
    void waitForGraphChange(Object event, int timeout);

    /**
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     * Return the number of on loan graph events, see get_graph_event().
     */
    int countGraphUsers();

    /**
     * Register the callback for parameter changes.
     * Repeated invocations of this function will overwrite previous callbacks
     *
     * @param User defined callback function, It is expected to atomically set parameters.
     */
    <T> void registerParamChangeCallback(Consumer<T> callback);

}

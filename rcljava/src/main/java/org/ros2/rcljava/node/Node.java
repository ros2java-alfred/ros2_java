/* Copyright 2016-2017 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.node;

import java.util.HashMap;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.time.WallTimer;
import org.ros2.rcljava.time.WallTimerCallback;

import builtin_interfaces.msg.Time;

import org.ros2.rcljava.Log;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.parameter.ParameterVariant;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;

import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.ParameterDescriptor;
import rcl_interfaces.msg.SetParametersResult;

/**
 * ROS2 Client API.
 *
 */
public interface Node {

    /**
     * Release all resource.
     */
    void dispose();

    /**
     * Get the name of the node.
     *
     * @return The name of the node.
     */
    String getName();

    /**
     * Get the namespace of the node.
     *
     * @return The namespace of the node.
     */
    String getNameSpace();

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topicName The topic for this publisher to publish on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Publisher instance of the created publisher.
     */
    <T extends Message> Publisher<T> createPublisher(
            final Class<T> message,
            final String topicName,
            final QoSProfile qos);

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topicName The topic for this publisher to publish on.
     * @param qosHistoryDepth The depth of the publisher message queue.
     * @return Publisher instance of the created publisher.
     */
    <T extends Message> Publisher<T> createPublisher(
            final Class<T> message,
            final String topicName,
            final int qosHistoryDepth);

    /**
     * Create and return a Publisher. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topicName The topic for this publisher to publish on.
     * @return Publisher instance of the created publisher.
     */
    <T extends Message> Publisher<T> createPublisher(
            final Class<T> message,
            final String topicName);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @param ignoreLocalPublications True to ignore local publications.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> message,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qos,
            final boolean ignoreLocalPublications);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> message,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qos);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qosHistoryDepth The depth of the subscription's incoming message queue.
     * @param ignoreLocalPublications True to ignore local publications.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> message,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final int qosHistoryDepth,
            final boolean ignoreLocalPublications);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qosHistoryDepth The depth of the subscription's incoming message queue.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> message,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final int qosHistoryDepth);

    /**
     * Create and return a Subscription. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> message,
            final String topicName,
            final SubscriptionCallback<T> callback);

    /**
     * Create a timer.
     *
     * @param period The time interval between triggers of the callback.
     * @param unit 	The unit of time interval.
     * @param callback The user-defined callback function.
     * @return WallTimer instance of the created timer.
     */
    WallTimer createWallTimer(
            final long period,
            final TimeUnit unit,
            final WallTimerCallback callback);

    /**
     * Create and return a Client.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param serviceName The service to subscribe on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Client instance of the service.
     */
    <T extends MessageService> Client<T> createClient(
            final Class<T> message,
            final String serviceName,
            final QoSProfile qos)
                    throws Exception;

    /**
     * Create and return a Client. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param serviceName The service to subscribe on.
     * @return Client instance of the service.
     */
    <T extends MessageService> Client<T> createClient(
            final Class<T> message,
            final String serviceName)
                    throws Exception ;

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
            final QoSProfile qos)
                    throws Exception;

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
            final ServiceCallback<?, ?> callback)
                    throws Exception;

    /**
     *
     * @param parameters
     * @return
     */
    List<SetParametersResult> setParameters(final List<ParameterVariant<?>> parameters);

    /**
     *
     * @param parameters
     * @return
     */
    SetParametersResult setParametersAtomically(final List<ParameterVariant<?>> parameters);

    /**
     *
     * @param name
     * @param value
     */
    <T> void setParameterIfNotSet(final String name, final T value);

    /**
     *
     * @param names
     * @return
     */
    List<ParameterVariant<?>> getParameters(final List<String> names);

    /**
     *
     * @param name The name of the parameter to get.
     * @return
     */
    ParameterVariant<?> getParameter(final String name);

    /**
     * Assign the value of the parameter if set into the parameter argument.
     * If the parameter was not set, then the "parameter" argument is never assigned a value.
     *
     * @param name The name of the parameter to get.
     * @param parameter The output where the value of the parameter should be assigned.
     * @return true if the parameter was set, false otherwise
     */
    boolean getParameter(final String name, ParameterVariant<?> parameter);

    /**
     * Get the parameter value, or the "alternative value" if not set, and assign it to "value".
     * If the parameter was not set, then the "value" argument is assigned the "alternative_value".
     * In all cases, the parameter remains not set after this function is called.
     *
     * @param name The name of the parameter to get.
     * @param value The output where the value of the parameter should be assigned.
     * @param alternativeParameter Value to be stored in output if the parameter was not set.
     * @return true if the parameter was set, false otherwise
     */
    boolean getParameterOr(final String name, ParameterVariant<?> value, ParameterVariant<?> alternativeParameter);

    /**
     *
     * @param names
     * @return
     */
    List<ParameterDescriptor> describeParameters(final List<String> names);

    /**
     *
     * @param names
     * @return
     */
    List<Class<?>> getParameterTypes(final List<String> names);

    /**
     *
     * @param names
     * @param depth
     * @return
     */
    ListParametersResult listParameters(final List<String> names, final int depth);

    /**
     * Register the callback for parameter changes.
     * Repeated invocations of this function will overwrite previous callbacks
     *
     * @param User defined callback function, It is expected to atomically set parameters.
     */
    <T extends Message> void registerParamChangeCallback(SubscriptionCallback<T> callback);

    /**
     *
     * @return
     */
    HashMap<String, List<String>> getTopicNamesAndTypes();

    /**
     *
     * @return
     */
    HashMap<String, List<String>> getServiceNamesAndTypes();

    /**
     *
     * @param topic
     * @return
     */
    int countPublishers(final String topic);

    /**
     *
     * @param topic
     * @return
     */
    int countSubscribers(final String topic);

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

///////////////////////////////// FROM rclcpp (C++ API) /////////////////////////////////

    List<String> getNodeNames();

    @Deprecated
    HashMap<String, List<String>> getTopicNamesAndTypes(boolean noDemangle);

    /**
     * Return the rcl_node_t node handle (non-const version).
     * @return
     */
    long getNodeHandle();

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
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     * Return the number of on loan graph events, see get_graph_event().
     */
    int countGraphUsers();

    List<Byte> getParametersTypes(List<String> names);

    List<String> getParametersNames();

    /**
     * @return All the @{link Subscription}s that were created by this instance.
     */
    Queue<Subscription<? extends Message>> getSubscriptions();

    /**
     * @return All the @{link Publisher}s that were created by this instance.
     */
    Queue<Publisher<? extends Message>> getPublishers();

    /**
     * Get list of Clients.
     * @return ArrayList of Clients
     */
    Queue<Client<? extends MessageService>> getClients();

    /**
     * Get list of Services.
     * @return ArrayList of Services
     */
    Queue<Service<? extends MessageService>> getServices();

    /**
     * @return All the @{link WallTimer}s that were created by this instance.
     */
    Queue<WallTimer> getWallTimers();

    Log getLog();

    Time getCurrentTime();
}

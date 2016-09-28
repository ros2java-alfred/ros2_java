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
package org.ros2.rcljava.node;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.logging.Logger;

import org.ros2.rcljava.Log;
import org.ros2.rcljava.QoSProfile;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceConsumer;
import org.ros2.rcljava.node.topic.Consumer;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;

import builtin_interfaces.msg.Time;

/**
 * Node ROS2.
 *
 * @author Esteve Fernandez <esteve@apache.org>
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class Node implements INode {

    private static Logger logger = Logger.getLogger(RCLJava.LOG_NAME);

    // Loading JNI library.
    static {
        RCLJava.loadLibrary("rcljavanode_Node__" + RCLJava.getRMWIdentifier());
    }

    /** Name of the node */
    private String name;

    /** Node handler */
    private final long nodeHandle;

    /** List of publishers */
    private final List<Publisher<?>> publishers;

    /** List of subscriptions */
    private final List<Subscription<?>> subscriptions;

    /** List of clients */
    private final List<Client<?>> clients;

    /** List of services */
    private final List<Service<?>> services;

    /** List of parameters */
    private final HashMap<String, Object> parameters;

    // Native call.
    private static native <T extends Message> long nativeCreatePublisherHandle(
            long nodeHandle, Class<T> cls, String topic, QoSProfile qos);

    private static native <T extends Message> long nativeCreateSubscriptionHandle(
            long nodeHandle, Class<T> cls, String topic, QoSProfile qos);

    private static native <T> long nativeCreateClientHandle(
            long nodeHandle, Class<T> cls, String service, QoSProfile qos);

    private static native <T> long nativeCreateServiceHandle(
            long nodeHandle, Class<T> cls, String service, QoSProfile qos);

    private static native void nativeDispose(long nodeHandle);

    private static native String nativeGetName(long nodeHandle);

    private static native int nativeCountPublishers(long nodeHandle, String topic);

    private static native int nativeCountSubscribers(long nodeHandle, String topic);

    private static native HashMap<String, String> getListTopics(long nodeHandle);


//    private static native  ; //rcl_service_server_is_available

    /**
     * Constructor of Node.
     * @param nodeHandle Handler to the node.
     */
    public Node(final long nodeHandle, final String nodeName) {
        this.name           = nodeName;
        this.nodeHandle     = nodeHandle;
        this.subscriptions  = new ArrayList<Subscription<?>>();
        this.publishers     = new ArrayList<Publisher<?>>();
        this.clients        = new ArrayList<Client<?>>();
        this.services       = new ArrayList<Service<?>>();
        this.parameters     = new HashMap<String, Object>();
    }

    /**
     * Release all resource.
     */
    @Override
    public void dispose() {
        logger.fine("Destroy Node stack : " + this.name);

        Node.nativeDispose(this.nodeHandle);
        this.publishers.clear();
        this.subscriptions.clear();
    }

    /**
     * Get the name of the node.
     *
     * @return The name of the node.
     */
    @Override
    public String getName() {
        String name = Node.nativeGetName(this.nodeHandle);

        if (!this.name.equals(name)) {
            logger.fine("Node name has change ! from " + this.name + " to " + name);
            this.name = name;
        }

        logger.fine("Get Node name stack : " + this.name);
        return name;
    }

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topic The topic for this publisher to publish on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Publisher instance of the created publisher.
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> message,
            final String topic,
            final QoSProfile qos) {

        logger.fine("Create Publisher : " + topic);
        long publisherHandle = Node.nativeCreatePublisherHandle(this.nodeHandle, message, topic, qos);

        Publisher<T> publisher = new Publisher<T>(this.nodeHandle, publisherHandle, message, topic, qos);
        RCLJava.publisherReferences.add(new WeakReference<Publisher<?>>(publisher));

        this.publishers.add(publisher);
        return publisher;
    }

    /**
     * Create and return a Publisher. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topic The topic for this publisher to publish on.
     * @return Publisher instance of the created publisher.
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> message,
            final String topic) {
        return this.createPublisher(message, topic, QoSProfile.PROFILE_DEFAULT);
    }

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
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> message,
            final String topic,
            final Consumer<T> callback,
            final QoSProfile qos) {

        logger.fine("Create Subscription : " + topic);
        long subscriptionHandle = Node.nativeCreateSubscriptionHandle(this.nodeHandle, message, topic, qos);

        Subscription<T> subscription = new Subscription<T>(
                this.nodeHandle,
                subscriptionHandle,
                message,
                topic,
                callback,
                qos);
        this.subscriptions.add(subscription);
        return subscription;
    }

    /**
     * Create and return a Subscription. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topic The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @return Subscription instance of the created subscription.
     */
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> message,
            final String topic,
            final Consumer<T> callback) {
        return this.createSubscription(message, topic, callback, QoSProfile.PROFILE_DEFAULT);
    }

    /**
     * Create and return a Client.
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service to subscribe on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Client instance of the service.
     */
    @Override
    public <T> Client<T> createClient(
            final Class<T> message,
            final String service,
            final QoSProfile qos) {

        logger.fine("Create Client : " + service);
        long clientHandle = Node.nativeCreateClientHandle(
                this.nodeHandle,
                message,
                service,
                qos);

        Client<T> client = new Client<T>(this.nodeHandle, clientHandle, service);
        this.clients.add(client);
        return client;
    }

    /**
     * Create and return a Client. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service to subscribe on.
     * @return Client instance of the service.
     */
    @Override
    public <T> Client<T> createClient(
            final Class<T> message,
            final String service) {
        return this.createClient(message, service, QoSProfile.PROFILE_DEFAULT);
    }

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
    @Override
    public <T> Service<T> createService(
            final Class<T> message,
            final String service,
            final ServiceConsumer<?, ?> callback,
            final QoSProfile qos) {

        logger.fine("Create Service : " + service);
        long serviceHandle = Node.nativeCreateServiceHandle(this.nodeHandle, message, service, qos);

        Service<T> srv = new Service<T>(this.nodeHandle, serviceHandle, service);
        this.services.add(srv);
        return srv;
    }

    /**
     * Create and return a Service. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service for this publisher to publish on.
     * @param callback The user-defined callback function.
     * @return Service instance of the service.
     */
    @Override
    public <T> Service<T> createService(
            final Class<T> message,
            final String service,
            final ServiceConsumer<?, ?> callback) {

        return this.createService(message, service, callback, QoSProfile.PROFILE_DEFAULT);
    }


    @Override
    public List<Object> setParameters(final List<Object> parameters) {
        //TODO
        throw new NotImplementedException();
//        return parameters;
    }

    @Override
    public List<Object> getParameters(final List<String> names) {
        //TODO
        throw new NotImplementedException();
//        return new ArrayList<Object>();
    }

    @Override
    public Object getParameter(final String name) {
        //TODO
        throw new NotImplementedException();
//        return name;
    }

    @Override
    public HashMap<String, String> getTopicNamesAndTypes() {
        HashMap<String, String> topics =  Node.getListTopics(this.nodeHandle);

        for (Entry<String, String> entry : topics.entrySet()) {
            logger.fine("\t - Topics: " + entry.getKey() + "\t Value: " + entry.getValue());
        }

        return topics;
    }

    @Override
    public int countPublishers(final String topic) {
        return Node.nativeCountPublishers(this.nodeHandle, topic);
    }

    @Override
    public int countSubscribers(final String topic) {
        return Node.nativeCountSubscribers(this.nodeHandle, topic);
    }

    /**
     * Return the rcl_node_t node handle (non-const version).
     * @return
     */
    @Override
    public long getRclNodeHandle() {
        return this.nodeHandle;
    }

    /**
     * Notify threads waiting on graph changes.
     *
     * Affects threads waiting on the notify guard condition, see:
     * get_notify_guard_condition(), as well as the threads waiting on graph
     * changes using a graph Event, see: wait_for_graph_change().
     *
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     *
     * @throws RCLBaseError (a child of that exception) when an rcl error occurs
     */
    @Override
    public void notifyGraphChange() {
        //TODO
        throw new NotImplementedException();
    }

    /** Notify any and all blocking node actions that shutdown has occurred. */
    @Override
    public void notifyShutdown() {
        //TODO
        throw new NotImplementedException();
    }

    /**
     * Return a graph event, which will be set anytime a graph change occurs.
     *
     * The graph Event object is a loan which must be returned.
     * The Event object is scoped and therefore to return the load just let it go
     * out of scope.
     */
    @Override
    public Object getGraphEvent() {
        //TODO
        throw new NotImplementedException();
//        return null;
    }

    /**
     * Wait for a graph event to occur by waiting on an Event to become set.
     *
     * The given Event must be acquire through the get_graph_event() method.
     *
     * @throws InvalidEventError if the given event is nullptr
     * @throws EventNotRegisteredError if the given event was not acquired with get_graph_event().
     */
    @Override
    public void waitForGraphChange(Object event, int timeout) {
        //TODO
        throw new NotImplementedException();
    }

    /**
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     * @return the number of on loan graph events, see get_graph_event().
     */
    @Override
    public int countGraphUsers() {
        //TODO
        throw new NotImplementedException();
//        return 0;
    }

    /**
     * Register the callback for parameter changes.
     * Repeated invocations of this function will overwrite previous callbacks
     *
     * @param User defined callback function, It is expected to atomically set parameters.
     */
    @Override
    public <T extends Message> void registerParamChangeCallback(Consumer<T> callback) {
        //TODO
        throw new NotImplementedException();
    }

    /**
     * Get list of Subscriptions.
     * @return ArrayList of Subscriptions
     */
    public List<Subscription<?>> getSubscriptions() {
        return this.subscriptions;
    }

    /**
     * Get list of Clients.
     * @return ArrayList of Clients
     */
    public List<Client<?>> getClients() {
        return this.clients;
    }

    /**
     * Get list of Services.
     * @return ArrayList of Services
     */
    public List<Service<?>> getServices() {
        return this.services;
    }

    public Time getCurrentTime() {
        return null;
    }

    public Log getLog() {
        return new Log(this);
    }
}

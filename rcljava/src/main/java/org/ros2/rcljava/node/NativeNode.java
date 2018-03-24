/* Copyright 2016-2017 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016-2017 Mickael Gaillard <mick.gaillard@gmail.com>
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
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import org.ros2.rcljava.ArgumentParser;
import org.ros2.rcljava.Log;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.exception.NotInitializedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.parameter.ParameterCallback;
import org.ros2.rcljava.node.parameter.ParameterService;
import org.ros2.rcljava.node.parameter.ParameterVariant;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.NativeClient;
import org.ros2.rcljava.node.service.NativeService;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.node.topic.NativePublisher;
import org.ros2.rcljava.node.topic.NativeSubscription;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.time.NativeWallTimer;
import org.ros2.rcljava.time.WallTimer;
import org.ros2.rcljava.time.WallTimerCallback;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import builtin_interfaces.msg.Time;
import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.Parameter;
import rcl_interfaces.msg.ParameterDescriptor;
import rcl_interfaces.msg.ParameterEvent;
import rcl_interfaces.msg.ParameterType;
import rcl_interfaces.msg.SetParametersResult;

/**
 * This class serves as a bridge between ROS2's rcl_node_t and RCLJava.
 * A Node must be created via @{link RCLJava#createNode(String)}
 */
public class NativeNode implements Node {

    private static final Logger logger = LoggerFactory.getLogger(NativeNode.class);

    private static final String ERROR_QOS       = "QOS can't be null";
    private static final String ERROR_MSG       = "Message Type can't be null.";
    private static final String ERROR_SRV       = "Service Type can't be null.";
    private static final String ERROR_TOPIC_MSG = "Topic can't be null.";
    private static final String ERROR_TOPIC_SRV = "Service name can't be null.";
    private static final String ERROR_CALLBACK  = "Callback can't be null.";

    // Loading JNI library.
    static {
        RCLJava.loadLibrary("rcljava_node_NativeNode"); //__" + RCLJava.getRMWIdentifier());
    }

//    /** Domain ID */
//    private int domainId = 0;

    /** Name of the node */
    private String name;

    /** Name space of the node. */
    private String nameSpace;

    /**
     * An integer that represents a pointer to the underlying ROS2 node
     * structure (rcl_node_t).
     */
    private final long nodeHandle;

    /**
     * All the @{link Subscription}s that have been created through this instance.
     */
    private final Queue<Subscription<? extends Message>> subscriptions;

    /**
     * All the @{link Publisher}s that have been created through this instance.
     */
    private final Queue<Publisher<? extends Message>> publishers;

    /**
     * All the @{link Service}s that have been created through this instance.
     */
    private final Queue<Service<? extends MessageService>> services;

    /**
     * All the @{link Client}s that have been created through this instance.
     */
    private final Queue<Client<? extends MessageService>> clients;

    /**
     * All the @{link WallTimer}s that have been created through this instance.
     */
    private final Queue<WallTimer> timers;

    /**
     *  List of parameters
     */
    private final Map<String, ParameterVariant<?>> parameters;

    /**
     * Parameter service.
     */
    private final ParameterService parameterService;
    private ParameterCallback parameterCallback ;

    public void onParamChange(ParameterCallback parameterCallback) {
        this.parameterCallback = parameterCallback;
    }

    /**
     * Log to ROS2.
     */
    private final Log logRos;

    /**
     * Constructor.
     *
     * @param defaultName name of node.
     */
    public NativeNode(final String defaultName) {
        this(null, defaultName, null);
    }

    /**
     * Constructor.
     *
     * @param nodeHandle A pointer to the underlying ROS2 node structure. Must not
     *     be zero.
     * @param namespace prefix path of node.
     * @param defaultName name of node.
     */
    public NativeNode(final String namespace, final String defaultName, final String[] args) {
        NativeNode.logger.debug("Create Node stack...");

        //TODO (Mickael) check if really needed !! (can be check by native load...)
        if (!RCLJava.isInitialized()) {
            throw new NotInitializedException();
        }

        ArgumentParser argParser = new ArgumentParser(namespace, defaultName, args);

        this.name           = argParser.getName();
        if (this.name==null || this.name.length() == 0) { throw new NullPointerException("Node name is needed !"); }

        this.nameSpace      = argParser.getNameSpace();
        this.parameters     = argParser.getParameters();

        // Initialize components.
        this.subscriptions  = new LinkedBlockingQueue<Subscription<?>>();
        this.publishers     = new LinkedBlockingQueue<Publisher<?>>();
        this.clients        = new LinkedBlockingQueue<Client<?>>();
        this.services       = new LinkedBlockingQueue<Service<? extends MessageService>>();
        this.timers         = new LinkedBlockingQueue<WallTimer>();

        // Initialize native component.
        this.nodeHandle = RCLJava.nativeCreateNodeHandle(this.name, this.nameSpace);
        if (this.nodeHandle==0) { throw new NullPointerException("Node Handle is not define !"); }

        NativeNode.logger.debug("Created Node stack : " + GraphName.getFullName(this.nameSpace, this.name));

        GraphName.addNode(this);
        this.parameterService = new ParameterService(this);
        this.logRos = new Log(this);
    }

    /**
     * Release all resource.
     */
    @Override
    public void dispose() {
        NativeNode.logger.debug("Destroy Node stack : " + this.name);
        this.parameterService.dispose();

        Queue<Client<?>> tmpClients = new LinkedBlockingQueue<Client<?>>(this.clients);
        for (Client<?> client : tmpClients) {
            client.dispose();
        }

        Queue<Publisher<?>> tmpPublishers = new LinkedBlockingQueue<Publisher<?>>(this.publishers);
        for (Publisher<?> publisher : tmpPublishers) {
            publisher.dispose();
        }

        Queue<Service<?>> tmpServices = new LinkedBlockingQueue<Service<?>>(this.services);
        for (Service<?> service : tmpServices) {
            service.dispose();
        }

        Queue<Subscription<?>> tmpSubscribers = new LinkedBlockingQueue<Subscription<?>>(this.subscriptions);
        for (Subscription<?> subscriber : tmpSubscribers) {
            subscriber.dispose();
        }

        NativeNode.nativeDispose(this.nodeHandle);
        GraphName.removeNode(this);
    }

    /**
     * Get the name of the node.
     *
     * @return The name of the node.
     */
    @Override
    public String getName() {
        String name = NativeNode.nativeGetName(this.nodeHandle);

        if (name != null && name.contains("/")) {
            if (this.nameSpace != null) {
                name = name.replace(this.nameSpace+"/", "");
            } else {
                name = name.replace("/", "");
            }
        }

        if (!this.name.equals(name)) {
            NativeNode.logger.debug("Node name has changed ! from " + this.name + " to " + name);
            this.name = name;
        }

        return name;
    }

    /**
     * Get the name-space of the node.
     *
     * @return The name-space of the node.
     */
    @Override
    public String getNameSpace() {
        return this.nameSpace;
    }

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topicName The topic for this publisher to publish on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Publisher instance of the created publisher.
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName,
            final QoSProfile qosProfile) {

        if (messageType == null) { throw new NullPointerException(ERROR_MSG); }
        if (topicName   == null) { throw new NullPointerException(ERROR_TOPIC_MSG); }
        if (qosProfile  == null) { throw new NullPointerException(ERROR_QOS); }

        String fqnTopic =  GraphName.getFullName(this, topicName, null);
        NativeNode.logger.debug("Create Publisher : " + fqnTopic);
        Publisher<T> publisher = null;

        if (GraphName.isValidTopic(fqnTopic)) {
            long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
            long publisherHandle = NativeNode.nativeCreatePublisherHandle(this.nodeHandle, messageType, fqnTopic, qosProfileHandle);
            RCLJava.disposeQoSProfile(qosProfileHandle);

            publisher = new NativePublisher<T>(this, publisherHandle, messageType, topicName, qosProfile);
        }

        return publisher;
    }

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topicName The topic for this publisher to publish on.
     * @param qosHistoryDepth The depth of the publisher message queue.
     * @return Publisher instance of the created publisher.
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> message,
            final String topicName,
            final int qosHistoryDepth) {

        QoSProfile qos = QoSProfile.SYSTEM_DEFAULT;
        //TODO fix the depth.
        return this.createPublisher(message, topicName, qos);
    }

    /**
     * Create and return a Publisher. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message class.
     * @param topicName The topic for this publisher to publish on.
     * @return Publisher instance of the created publisher.
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName) {
        return this.createPublisher(messageType, topicName, QoSProfile.DEFAULT);
    }

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
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qosProfile,
            final boolean ignoreLocalPublication ) {  //TODO use it

        if (messageType == null) { throw new NullPointerException(ERROR_MSG); }
        if (topicName   == null) { throw new NullPointerException(ERROR_TOPIC_MSG); }
        if (callback    == null) { throw new NullPointerException(ERROR_CALLBACK); }
        if (qosProfile  == null) { throw new NullPointerException(ERROR_QOS); }

        String fqnTopic =  GraphName.getFullName(this, topicName, null);
        NativeNode.logger.debug("Create Subscription : " + fqnTopic);
        Subscription<T> subscription = null;

        if (GraphName.isValidTopic(fqnTopic)) {
            long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
            long subscriptionHandle = NativeNode.nativeCreateSubscriptionHandle(this.nodeHandle, messageType, fqnTopic, qosProfileHandle);
            RCLJava.disposeQoSProfile(qosProfileHandle);

            subscription = new NativeSubscription<T>(
                    this,
                    subscriptionHandle,
                    messageType,
                    topicName,
                    callback,
                    qosProfile);
        }

        return subscription;
    }

    @Override
    public <T extends Message> Subscription<T> createSubscription(Class<T> message, String topicName,
            SubscriptionCallback<T> callback, QoSProfile qos) {
        return this.createSubscription(message, topicName, callback, qos, false);
    }

    @Override
    public <T extends Message> Subscription<T> createSubscription(Class<T> message, String topicName,
            SubscriptionCallback<T> callback, int qosHistoryDepth) {
        return this.createSubscription(message, topicName, callback, qosHistoryDepth, false);
    }

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
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final int qosHistoryDepth,
            final boolean ignoreLocalPublication) {  //TODO use it

        QoSProfile qos = QoSProfile.SYSTEM_DEFAULT;
        //TODO fix the depth.
        return this.createSubscription(messageType, topicName, callback, qos, ignoreLocalPublication);
    }

    /**
     * Create and return a Subscription. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @return Subscription instance of the created subscription.
     */
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback) {

        return this.createSubscription(messageType, topicName, callback, QoSProfile.DEFAULT, false);
    }

    /**
     * Create a timer.
     *
     * @param period The time interval between triggers of the callback.
     * @param unit 	The unit of time interval.
     * @param callback The user-defined callback function.
     * @return WallTimer instance of the created timer.
     */
    @Override
    public WallTimer createWallTimer(final long period, final TimeUnit unit, final WallTimerCallback callback) {
        long timerPeriodNS = TimeUnit.NANOSECONDS.convert(period, unit);
        long timerHandle = NativeNode.nativeCreateTimerHandle(timerPeriodNS);

        WallTimer timer = new NativeWallTimer(new WeakReference<Node>(this), timerHandle, callback, timerPeriodNS);
        this.timers.add(timer);
        return timer;
    }

    /**
     * Create and return a Client.
     *
     * @param <T> Message definition.
     * @param serviceType Service Class
     * @param serviceName The service to subscribe on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Client instance of the service.
     * @throws SecurityException
     * @throws NoSuchFieldException
     * @throws IllegalAccessException
     * @throws IllegalArgumentException
     * @throws NoSuchMethodException
     */
    @SuppressWarnings("unchecked")
    @Override
    public <T extends MessageService> Client<T> createClient(
            final Class<T> serviceType,
            final String serviceName,
            final QoSProfile qosProfile) {

        if (serviceType == null) { throw new NullPointerException(ERROR_SRV); }
        if (serviceName == null) { throw new NullPointerException(ERROR_TOPIC_SRV); }
        if (qosProfile  == null) { throw new NullPointerException(ERROR_QOS); }

        String fqnService =  GraphName.getFullName(this, serviceName, null);
        NativeNode.logger.debug("Create Client : " + fqnService);
        Client<T> client = null;

        if (GraphName.isValidTopic(fqnService)) {
            Class<? extends Message> requestType = null;
            Method requestFromJavaConverterMethod = null, requestToJavaConverterMethod = null;
            long requestFromJavaConverterHandle = 0L, requestToJavaConverterHandle = 0L;

            Class<? extends Message> responseType = null;
            Method responseFromJavaConverterMethod = null, responseToJavaConverterMethod = null;
            long responseFromJavaConverterHandle = 0L, responseToJavaConverterHandle = 0L;

            try {
                requestType = (Class<? extends Message>)serviceType.getField("RequestType").get(null);

                requestFromJavaConverterMethod = requestType.getDeclaredMethod("getFromJavaConverter", (Class<?> []) null);
                requestFromJavaConverterHandle = (Long)requestFromJavaConverterMethod.invoke(null, (Object []) null);

                requestToJavaConverterMethod = requestType.getDeclaredMethod("getToJavaConverter", (Class<?> []) null);
                requestToJavaConverterHandle = (Long)requestToJavaConverterMethod.invoke(null, (Object []) null);

                responseType = (Class<? extends Message>)serviceType.getField("ResponseType").get(null);

                responseFromJavaConverterMethod = responseType.getDeclaredMethod("getFromJavaConverter", (Class<?> []) null);
                responseFromJavaConverterHandle = (Long)responseFromJavaConverterMethod.invoke(null, (Object []) null);

                responseToJavaConverterMethod = responseType.getDeclaredMethod("getToJavaConverter", (Class<?> []) null);
                responseToJavaConverterHandle = (Long)responseToJavaConverterMethod.invoke(null, (Object []) null);
            } catch (Exception e) {
                // TODO: handle exception
            }

            long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
            long clientHandle = NativeNode.nativeCreateClientHandle(
                    this.nodeHandle,
                    serviceType,
                    fqnService,
                    qosProfileHandle);
            RCLJava.disposeQoSProfile(qosProfileHandle);

            client = new NativeClient<T>(
                    new WeakReference<Node>(this),
                    clientHandle,
                    serviceType,
                    serviceName,
                    requestType,
                    responseType,
                    requestFromJavaConverterHandle,
                    requestToJavaConverterHandle,
                    responseFromJavaConverterHandle,
                    responseToJavaConverterHandle);
        }

        return client;
    }

    /**
     * Create and return a Client. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service to subscribe on.
     * @return Client instance of the service.
     * @throws SecurityException
     * @throws NoSuchFieldException
     * @throws IllegalAccessException
     * @throws IllegalArgumentException
     * @throws NoSuchMethodException
     */
    @Override
    public <T extends MessageService> Client<T> createClient(
            final Class<T> serviceType,
            final String serviceName) {
        return this.createClient(serviceType, serviceName, QoSProfile.SERVICES_DEFAULT);
    }

    /**
     * Create and return a Service.
     *
     * @param <T> Message definition.
     * @param serviceType Service Class
     * @param serviceName The service for this publisher to publish on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Service instance of the service.
     * @throws SecurityException
     * @throws NoSuchFieldException
     * @throws IllegalAccessException
     * @throws IllegalArgumentException
     * @throws NoSuchMethodException
     */
    @SuppressWarnings("unchecked")
    @Override
    public <T extends MessageService> Service<T> createService(
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback,
            final QoSProfile qosProfile
            ) {

        if (serviceType == null) { throw new NullPointerException(ERROR_SRV); }
        if (serviceName == null) { throw new NullPointerException(ERROR_TOPIC_SRV); }
        if (callback    == null) { throw new NullPointerException(ERROR_CALLBACK); }
        if (qosProfile  == null) { throw new NullPointerException(ERROR_QOS); }

        String fqnService =  GraphName.getFullName(this, serviceName, null);
        NativeNode.logger.debug("Create Service : " + fqnService);
        Service<T> service = null;

        if (GraphName.isValidTopic(fqnService)) {
//            long serviceHandle = Node.nativeCreateServiceHandle(this.nodeHandle, message, service, qos);
//            Service<T> srv = new Service<T>(this.nodeHandle, serviceHandle, service);
            Class<? extends Message> requestType = null;
            Method requestFromJavaConverterMethod = null, requestToJavaConverterMethod = null;
            long requestFromJavaConverterHandle = 0L, requestToJavaConverterHandle = 0L;

            Class<? extends Message> responseType = null;
            Method responseFromJavaConverterMethod = null, responseToJavaConverterMethod = null;
            long responseFromJavaConverterHandle = 0L, responseToJavaConverterHandle = 0L;

            try {
                requestType = (Class<? extends Message>)serviceType.getField("RequestType").get(null);

                requestFromJavaConverterMethod = requestType.getDeclaredMethod("getFromJavaConverter", (Class<?> []) null);
                requestFromJavaConverterHandle = (Long)requestFromJavaConverterMethod.invoke(null, (Object []) null);

                requestToJavaConverterMethod = requestType.getDeclaredMethod("getToJavaConverter", (Class<?> []) null);
                requestToJavaConverterHandle = (Long)requestToJavaConverterMethod.invoke(null, (Object []) null);

                responseType = (Class<? extends Message>)serviceType.getField("ResponseType").get(null);

                responseFromJavaConverterMethod = responseType.getDeclaredMethod("getFromJavaConverter", (Class<?> []) null);
                responseFromJavaConverterHandle = (Long)responseFromJavaConverterMethod.invoke(null, (Object []) null);

                responseToJavaConverterMethod = responseType.getDeclaredMethod("getToJavaConverter", (Class<?> []) null);
                responseToJavaConverterHandle = (Long)responseToJavaConverterMethod.invoke(null, (Object []) null);
            } catch (Exception e) {
                // TODO: handle exception
            }

            long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
            long serviceHandle = NativeNode.nativeCreateServiceHandle(this.nodeHandle, serviceType, serviceName, qosProfileHandle);
            RCLJava.disposeQoSProfile(qosProfileHandle);

            service = new NativeService<T>(this,
                    serviceHandle,
                    serviceType,
                    fqnService,
                    callback,
                    requestType,
                    responseType,
                    requestFromJavaConverterHandle,
                    requestToJavaConverterHandle,
                    responseFromJavaConverterHandle,
                    responseToJavaConverterHandle);
        }

        return service;
    }

    /**
     * Create and return a Service. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param message Message Class
     * @param service The service for this publisher to publish on.
     * @param callback The user-defined callback function.
     * @return Service instance of the service.
     * @throws SecurityException
     * @throws NoSuchFieldException
     * @throws IllegalAccessException
     * @throws IllegalArgumentException
     * @throws NoSuchMethodException
     */
    @Override
    public <T extends MessageService> Service<T> createService(
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback) {

        return this.createService(serviceType, serviceName, callback, QoSProfile.SERVICES_DEFAULT);
    }

    @Override
    public List<SetParametersResult> setParameters(final List<ParameterVariant<?>> parameters) {
        List<SetParametersResult> results = new ArrayList<SetParametersResult>();

        ArrayList<ParameterVariant<?>> container = new ArrayList<ParameterVariant<?>>(1);
        for (ParameterVariant<?> parameterVariantRequest : parameters) {
            container.clear();
            container.add(parameterVariantRequest);

            SetParametersResult result = this.setParametersAtomically(container);
            results.add(result);
        }

        return results;
    }

    @Override
    public SetParametersResult setParametersAtomically(final List<ParameterVariant<?>> parameters) {
        ParameterEvent parameter_event = new ParameterEvent();

        // TODO (jacquelinekay) Check handle parameter constraints
        SetParametersResult result = new SetParametersResult();

        if (this.parameterCallback != null) {
            result = this.parameterCallback.onParamChange(parameters);
        } else {
            result.setSuccessful(true);
        }

        if (result.getSuccessful()){
            for (ParameterVariant<?> paramVarReq : parameters) {
                Parameter parameter = paramVarReq.toParameter();

                if (!this.parameters.containsKey(paramVarReq.getName())) {
                    if (parameter.getValue().getType() != ParameterType.PARAMETER_NOT_SET) {
                        parameter_event.getNewParameters().add(parameter);
                    }
                } else {
                    if (parameter.getValue().getType() != ParameterType.PARAMETER_NOT_SET) {
                        parameter_event.getChangedParameters().add(parameter);
                    } else {
                        parameter_event.getDeletedParameters().add(parameter);
                    }
                }
                this.parameters.put(paramVarReq.getName(), paramVarReq);
            }
        }

        if (this.parameterService != null) {
            this.parameterService.notifyAddEvent(parameter_event);
        }

        return result;
    }

    @Override
    public <T> void setParameterIfNotSet(final String name, final T value) {
        throw new NotImplementedException();
    }

    @Override
    public List<ParameterVariant<?>> getParameters(final List<String> names) {
        List<ParameterVariant<?>>  result = new ArrayList<ParameterVariant<?>>();

        for (String name : names) {
            ParameterVariant<?> param = this.getParameter(name);
            if (param != null) {
                result.add(param);
            }
        }

        return result;
    }

    @Override
    public ParameterVariant<?> getParameter(final String name) {
        ParameterVariant<?> result = null;

        if (this.parameters.containsKey(name)) {
            result = this.parameters.get(name);
        }

        return result;
    }

    @Override
    public boolean getParameter(final String name, ParameterVariant<?> parameter) {
        throw new NotImplementedException();
    }

    @Override
    public boolean getParameterOr(final String name, ParameterVariant<?> value, ParameterVariant<?> alternativeParameter) {
        throw new NotImplementedException();
    }

    @Override
    public List<Class<?>> getParameterTypes(List<String> names) {
        throw new NotImplementedException();
    }

    @Override
    public ListParametersResult listParameters(List<String> names, int depth) {
        throw new NotImplementedException();
    }

    @Override
    public List<ParameterDescriptor> describeParameters(final List<String> names) {
        throw new NotImplementedException();
    }

    @Override
    public List<String> getParametersNames() {
        return new ArrayList<String>(this.parameters.keySet());
    }

    @Override
    public List<Byte> getParametersTypes(List<String> names) {
        List<Byte> result = new ArrayList<Byte>();

        for (String name : names) {
            if (this.parameters.containsKey(name)) {
                result.add(this.parameters.get(name).toParameterValue().getType());
            }
        }

        return result;
    }

    @Override
    public HashMap<String, List<String>> getTopicNamesAndTypes() {
        return this.getTopicNamesAndTypes(false);
    }

    @Override
    public HashMap<String, List<String>> getTopicNamesAndTypes(boolean noDemangle) {
        HashMap<String, List<String>> topics =  NativeNode.nativeGetListTopics(this.nodeHandle, noDemangle);

        for (Entry<String, List<String>> entry : topics.entrySet()) {
            NativeNode.logger.debug("\t - Topics: " + entry.getKey() + "\t Value: " + entry.getValue());
        }

        return topics;
    }

    @Override
    public HashMap<String, List<String>> getServiceNamesAndTypes() {
        HashMap<String, List<String>> services =  NativeNode.nativeGetListServices(this.nodeHandle);

        for (Entry<String, List<String>> entry : services.entrySet()) {
            NativeNode.logger.debug("\t - Service: " + entry.getKey() + "\t Value: " + entry.getValue());
        }

        return services;
    }

    @Override
    public List<String> getNodeNames() {
        return NativeNode.nativeGetNodeNames(this.nodeHandle);
    }

    @Override
    public int countPublishers(final String topic) {
        return NativeNode.nativeCountPublishers(this.nodeHandle, topic);
    }

    @Override
    public int countSubscribers(final String topic) {
        return NativeNode.nativeCountSubscribers(this.nodeHandle, topic);
    }

    /**
     * Return the rcl_node_t node handle (non-const version).
     * @return
     */
    @Override
    public long getNodeHandle() {
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
    public <T extends Message> void registerParamChangeCallback(SubscriptionCallback<T> callback) {
        //TODO
        throw new NotImplementedException();
    }

    /**
     * @return All the @{link Subscription}s that were created by this instance.
     */
    @Override
    public Queue<Subscription<? extends Message>> getSubscriptions() {
        return this.subscriptions;
    }

    /**
     * @return All the @{link Publisher}s that were created by this instance.
     */
    @Override
    public Queue<Publisher<? extends Message>> getPublishers() {
      return this.publishers;
    }

    /**
     * Get list of Clients.
     * @return ArrayList of Clients
     */
    @Override
    public Queue<Client<? extends MessageService>> getClients() {
        return this.clients;
    }

    /**
     * Get list of Services.
     * @return ArrayList of Services
     */
    @Override
    public Queue<Service<? extends MessageService>> getServices() {
        return this.services;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Queue<WallTimer> getWallTimers() {
        return this.timers;
    }

    @Override
    public Time getCurrentTime() {
        long lt = System.currentTimeMillis();
        Time t = new Time();
        t.setSec((int) (lt / 1e3));
        t.setNanosec((int) ((lt % 1e3) * 1e6));
        return t;
    }

    @Override
    public Log getLog() {
        return this.logRos;
    }

    @Override
    public void close() throws Exception {
        this.dispose();
    }

 // Native call.
    /**
     * Create a ROS2 publisher (rcl_publisher_t) and return a pointer to
     *     it as an integer.
     *
     * @param <T> The type of the messages that will be published by the
     *     created @{link Publisher}.
     * @param nodeHandle A pointer to the underlying ROS2 node structure.
     * @param messageType The class of the messages that will be published by the
     *     created @{link Publisher}.
     * @param topic The topic to which the created @{link Publisher} will
     *     publish messages.
     * @param qosProfileHandle A pointer to the underlying ROS2 QoS profile
     *     structure.
     * @return A pointer to the underlying ROS2 publisher structure.
     */
    public static native <T extends Message> long nativeCreatePublisherHandle(
            long nodeHandle, Class<T> messageType, String topic, long qosProfileHandle);

    /**
     * Create a ROS2 subscription (rcl_subscription_t) and return a pointer to
     *     it as an integer.
     *
     * @param <T> The type of the messages that will be received by the
     *     created @{link Subscription}.
     * @param nodeHandle A pointer to the underlying ROS2 node structure.
     * @param messageType The class of the messages that will be received by the
     *     created @{link Subscription}.
     * @param topic The topic from which the created @{link Subscription} will
     *     receive messages.
     * @param qosProfileHandle A pointer to the underlying ROS2 QoS profile
     *     structure.
     * @return A pointer to the underlying ROS2 subscription structure.
     */
    public static native <T> long nativeCreateSubscriptionHandle(
            long nodeHandle, Class<T> messageType, String topic, long qosProfileHandle);

    public static native <T> long nativeCreateClientHandle(
            long nodeHandle, Class<T> cls, String serviceName, long qosProfileHandle);

    public static native <T> long nativeCreateServiceHandle(
            long nodeHandle, Class<T> cls, String serviceName, long qosProfileHandle);

    public static native long nativeCreateTimerHandle(long timerPeriod); //TODO move to RCLJava

    public static native void nativeDispose(long nodeHandle);

    public static native String nativeGetName(long nodeHandle);

    public static native int nativeCountPublishers(long nodeHandle, String topic);

    public static native int nativeCountSubscribers(long nodeHandle, String topic);

    public static native HashMap<String, List<String>> nativeGetListTopics(long nodeHandle, boolean noDemangle);

    public static native HashMap<String, List<String>> nativeGetListServices(long nodeHandle);

    public static native List<String> nativeGetNodeNames(long nodeHandle);

//    public static native  ; //rcl_service_server_is_available
}

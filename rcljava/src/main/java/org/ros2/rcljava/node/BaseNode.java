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

package org.ros2.rcljava.node;

import java.time.Clock;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros2.rcljava.ArgumentParser;
import org.ros2.rcljava.Logger;
import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.parameter.ParameterCallback;
import org.ros2.rcljava.node.parameter.ParameterService;
import org.ros2.rcljava.node.parameter.ParameterVariant;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.qos.QoSProfile;
import org.ros2.rcljava.time.WallTimer;
import org.slf4j.LoggerFactory;

import builtin_interfaces.msg.Time;
import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.Parameter;
import rcl_interfaces.msg.ParameterDescriptor;
import rcl_interfaces.msg.ParameterEvent;
import rcl_interfaces.msg.ParameterType;
import rcl_interfaces.msg.SetParametersResult;

/**
 *
 */
public abstract class BaseNode implements Node {

    private static final org.slf4j.Logger logger = LoggerFactory.getLogger(BaseNode.class);

    protected static final String ERROR_QOS       = "QOS can't be null";
    protected static final String ERROR_MSG       = "Message Type can't be null.";
    protected static final String ERROR_SRV       = "Service Type can't be null.";
    protected static final String ERROR_TOPIC_MSG = "Topic can't be null.";
    protected static final String ERROR_TOPIC_SRV = "Service name can't be null.";
    protected static final String ERROR_CALLBACK  = "Callback can't be null.";

//  /** Domain ID */
//  private int domainId = 0;

    /** Name of the node */
    protected String name;

    /** Name space of the node. */
    protected String nameSpace;

    protected int domainId;

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
     * Parameter service.
     */
    private ParameterService parameterService;

    /**
     *  List of parameters
     */
    private final Map<String, ParameterVariant<?>> parameters;

    /**
     * Log to ROS2.
     */
    private Logger rosLogger;

    private ParameterCallback parameterCallback ;

    public BaseNode(final String namespace, final String defaultName, final String... args) {
        BaseNode.logger.debug("Create Node stack...");

        final ArgumentParser argParser = new ArgumentParser(namespace, defaultName, args);

        this.name           = argParser.getName();
        if (this.name==null || this.name.length() == 0) { throw new NullPointerException("Node name is needed !"); }

        this.nameSpace      = argParser.getNameSpace();
        this.parameters     = argParser.getParameters();
        this.domainId       = argParser.getDomainId();

        // Initialize components.
        this.subscriptions  = new LinkedBlockingQueue<Subscription<?>>();
        this.publishers     = new LinkedBlockingQueue<Publisher<?>>();
        this.clients        = new LinkedBlockingQueue<Client<?>>();
        this.services       = new LinkedBlockingQueue<Service<? extends MessageService>>();
        this.timers         = new LinkedBlockingQueue<WallTimer>();

        GraphName.addNode(this);
    }

    public void startParameterService() {
        this.parameterService = new ParameterService(this);
        this.rosLogger = new Logger(this);
    }

    public void onParamChange(final ParameterCallback parameterCallback) {
        this.parameterCallback = parameterCallback;
    }

    /* (non-Javadoc)
     * @see java.lang.AutoCloseable#close()
     */
    @Override
    public void close() throws Exception {
        this.dispose();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#dispose()
     */
    @Override
    public void dispose() {
        BaseNode.logger.debug("Destroy Node stack : " + this.name);
        this.parameterService.dispose();

        final Queue<Client<?>> tmpClients = new LinkedBlockingQueue<Client<?>>(this.clients);
        for (final Client<?> client : tmpClients) {
            client.dispose();
        }

        final Queue<Publisher<?>> tmpPublishers = new LinkedBlockingQueue<Publisher<?>>(this.publishers);
        for (final Publisher<?> publisher : tmpPublishers) {
            publisher.dispose();
        }

        final Queue<Service<?>> tmpServices = new LinkedBlockingQueue<Service<?>>(this.services);
        for (final Service<?> service : tmpServices) {
            service.dispose();
        }

        final Queue<Subscription<?>> tmpSubscribers = new LinkedBlockingQueue<Subscription<?>>(this.subscriptions);
        for (final Subscription<?> subscriber : tmpSubscribers) {
            subscriber.dispose();
        }

        GraphName.removeNode(this);
    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#getName()
    */
    @Override
    public String getName() {
        return this.name;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getNameSpace()
     */
    @Override
    public String getNameSpace() {
        return this.nameSpace;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createPublisher(java.lang.Class, java.lang.String, int)
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName,
            final int qosHistoryDepth) {

        final QoSProfile qos = QoSProfile.SYSTEM_DEFAULT;
        //TODO fix the depth.
        return this.createPublisher(messageType, topicName, qos);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createPublisher(java.lang.Class, java.lang.String)
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName) {

        return this.createPublisher(messageType, topicName, QoSProfile.DEFAULT);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createSubscription(java.lang.Class, java.lang.String, org.ros2.rcljava.node.topic.SubscriptionCallback, org.ros2.rcljava.qos.QoSProfile)
     */
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qos) {

        return this.createSubscription(messageType, topicName, callback, qos, false);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createSubscription(java.lang.Class, java.lang.String, org.ros2.rcljava.node.topic.SubscriptionCallback, int, boolean)
     */
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final int qosHistoryDepth,
            final boolean ignoreLocalPublications) {

        final QoSProfile qos = QoSProfile.SYSTEM_DEFAULT;
        //TODO fix the depth.
        return this.createSubscription(messageType, topicName, callback, qos, ignoreLocalPublications);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createSubscription(java.lang.Class, java.lang.String, org.ros2.rcljava.node.topic.SubscriptionCallback, int)
     */
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final int qosHistoryDepth) {

        return this.createSubscription(messageType, topicName, callback, qosHistoryDepth, false);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createSubscription(java.lang.Class, java.lang.String, org.ros2.rcljava.node.topic.SubscriptionCallback)
     */
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback) {

        return this.createSubscription(messageType, topicName, callback, QoSProfile.DEFAULT, false);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createClient(java.lang.Class, java.lang.String)
     */
    @Override
    public <T extends MessageService> Client<T> createClient(
            final Class<T> serviceType,
            final String serviceName) {

        return this.createClient(serviceType, serviceName, QoSProfile.SERVICES_DEFAULT);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#createService(java.lang.Class, java.lang.String, org.ros2.rcljava.node.service.ServiceCallback)
     */
    @Override
    public <T extends MessageService> Service<T> createService(
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback) {

        return this.createService(serviceType, serviceName, callback, QoSProfile.SERVICES_DEFAULT);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#setParameters(java.util.List)
     */
    @Override
    public List<SetParametersResult> setParameters(final List<ParameterVariant<?>> parameters) {
        final List<SetParametersResult> results = new ArrayList<SetParametersResult>();

        final ArrayList<ParameterVariant<?>> container = new ArrayList<ParameterVariant<?>>(1);
        for (final ParameterVariant<?> parameterVariantRequest : parameters) {
            container.clear();
            container.add(parameterVariantRequest);

            final SetParametersResult result = this.setParametersAtomically(container);
            results.add(result);
        }

        return results;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#setParametersAtomically(java.util.List)
     */
    @Override
    public SetParametersResult setParametersAtomically(final List<ParameterVariant<?>> parameters) {
        final ParameterEvent parameter_event = new ParameterEvent();

        // TODO (jacquelinekay) Check handle parameter constraints
        SetParametersResult result = new SetParametersResult();

        if (this.parameterCallback != null) {
            result = this.parameterCallback.onParamChange(parameters);
        } else {
            result.setSuccessful(true);
        }

        if (result.getSuccessful()){
            for (final ParameterVariant<?> paramVarReq : parameters) {
                final Parameter parameter = paramVarReq.toParameter();

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

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#setParameterIfNotSet(java.lang.String, java.lang.Object)
     */
    @Override
    public <T> void setParameterIfNotSet(final String name, final T value) {
        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getParameters(java.util.List)
     */
    @Override
    public List<ParameterVariant<?>> getParameters(final List<String> names) {
        final List<ParameterVariant<?>>  result = new ArrayList<ParameterVariant<?>>();

        for (final String name : names) {
            final ParameterVariant<?> param = this.getParameter(name);

            if (param != null) {
                result.add(param);
            }
        }

        return result;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getParameter(java.lang.String)
     */
    @Override
    public ParameterVariant<?> getParameter(final String name) {
        ParameterVariant<?> result = null;

        if (this.parameters.containsKey(name)) {
            result = this.parameters.get(name);
        }

        return result;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getParameter(java.lang.String, org.ros2.rcljava.node.parameter.ParameterVariant)
     */
    @Override
    public boolean getParameter(final String name, final ParameterVariant<?> parameter) {
        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getParameterOr(java.lang.String, org.ros2.rcljava.node.parameter.ParameterVariant, org.ros2.rcljava.node.parameter.ParameterVariant)
     */
    @Override
    public boolean getParameterOr(final String name, final ParameterVariant<?> value, final ParameterVariant<?> alternativeParameter) {
        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#describeParameters(java.util.List)
     */
    @Override
    public List<ParameterDescriptor> describeParameters(final List<String> names) {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getParameterTypes(java.util.List)
     */
    @Override
    public List<Class<?>> getParameterTypes(final List<String> names) {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#listParameters(java.util.List, int)
     */
    @Override
    public ListParametersResult listParameters(final List<String> names, int depth) {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#registerParamChangeCallback(org.ros2.rcljava.node.topic.SubscriptionCallback)
    */
    @Override
    public <T extends Message> void registerParamChangeCallback(final SubscriptionCallback<T> callback) {
        // TODO Auto-generated method stub

    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getTopicNamesAndTypes()
     */
    @Override
    public Map<String, List<String>> getTopicNamesAndTypes() {
        return this.getTopicNamesAndTypes(false);
    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#getServiceNamesAndTypes()
    */
    @Override
    public Map<String, List<String>> getServiceNamesAndTypes() {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#countPublishers(java.lang.String)
    */
    @Override
    public int countPublishers(final String topic) {
        int result = 0;

        for (final Publisher<? extends Message> publisher : this.publishers) {
            if (publisher.getTopicName().equals(topic)) {
                ++result;
            }
        }

        return result;
    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#countSubscribers(java.lang.String)
    */
    @Override
    public int countSubscribers(final String topic) {
        int result = 0;

        for (final Subscription<? extends Message> subscription : this.subscriptions) {
            if (subscription.getTopicName().equals(topic)) {
                ++result;
            }
        }

        return result;
    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#getGraphEvent()
    */
    @Override
    public Object getGraphEvent() {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#waitForGraphChange(java.lang.Object, int)
    */
    @Override
    public void waitForGraphChange(final Object event, final int timeout) {
        // TODO Auto-generated method stub

    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#getTopicNamesAndTypes(boolean)
    */
//    @Override
//    public HashMap<String, List<String>> getTopicNamesAndTypes(boolean noDemangle) {
//        // TODO Auto-generated method stub
//        return null;
//    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#notifyGraphChange()
    */
    @Override
    public void notifyGraphChange() {
        // TODO Auto-generated method stub

    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#notifyShutdown()
    */
    @Override
    public void notifyShutdown() {
        // TODO Auto-generated method stub

    }

    /* (non-Javadoc)
    * @see org.ros2.rcljava.node.Node#countGraphUsers()
    */
    @Override
    public int countGraphUsers() {
        // TODO Auto-generated method stub
        return 0;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getParametersTypes(java.util.List)
     */
    @Override
    public List<Byte> getParametersTypes(final List<String> names) {
        final List<Byte> result = new ArrayList<Byte>();

        for (final String name : names) {
            if (this.parameters.containsKey(name)) {
                result.add(this.parameters.get(name).toParameterValue().getType());
            }
        }

        return result;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getParametersNames()
     */
    public List<String> getParametersNames() {
        return new ArrayList<String>(this.parameters.keySet());
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getSubscriptions()
     */
    @Override
    public Queue<Subscription<? extends Message>> getSubscriptions() {
        return this.subscriptions;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getPublishers()
     */
    @Override
    public Queue<Publisher<? extends Message>> getPublishers() {
        return this.publishers;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getClients()
     */
    @Override
    public Queue<Client<? extends MessageService>> getClients() {
        return this.clients;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getServices()
     */
    @Override
    public Queue<Service<? extends MessageService>> getServices() {
        return this.services;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getWallTimers()
     */
    @Override
    public Queue<WallTimer> getWallTimers() {
        return this.timers;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getLogger()
     */
    @Override
    public Logger getLogger() {
        return this.rosLogger;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getLoggerName()
     */
    @Override
    public String getLoggerName() {
        return this.rosLogger.getName();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getCurrentTime()
     */
    @Override
    public Time now() {
        final long lt = System.currentTimeMillis();
        final Time t = new Time();
        t.setSec((int) (lt / 1e3));
        t.setNanosec((int) ((lt % 1e3) * 1e6));
        return t;
    }

    @Override
    public Clock getClock() {
        // TODO Auto-generated method stub
        return null;
    }

}

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

package org.ros2.rcljava.node.topic;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.qos.QoSProfile;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @param <T>
 */
public abstract class BaseSubscription<T extends Message> implements Subscription<T> {

    private static final Logger logger = LoggerFactory.getLogger(BaseSubscription.class);

    /** Node owner. */
    private final Node ownerNode;

    /** The class of the messages that this subscription may receive. */
    private final Class<T> messageType;

    /** The topic to which this subscription is subscribed. */
    private final String topicName;

    /** The callback function that will be triggered when a new message is received. */
    private final SubscriptionCallback<T> callback;

    /** Quality of Service profile. */
    private final QoSProfile qosProfile;

    /**
     *
     * @param node
     * @param messageType
     * @param topic
     * @param callback
     * @param qosProfile
     */
    public BaseSubscription(
            final Node node,
            final Class<T> messageType,
            final String topic,
            final SubscriptionCallback<T> callback,
            final QoSProfile qosProfile) {

        if (node == null) { throw new RuntimeException("Need to provide active node with handle object"); }
        this.ownerNode = node;

        this.messageType = messageType;
        this.topicName   = topic;
        this.callback    = callback;
        this.qosProfile  = qosProfile;

        BaseSubscription.logger.debug("Create Publisher of topic : " + this.topicName);
        this.ownerNode.getSubscriptions().add(this);
    }


    /* (non-Javadoc)
     * @see java.lang.AutoCloseable#close()
     */
    @Override
    public void close() throws Exception {
        this.dispose();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Subscription#dispose()
     */
    @Override
    public void dispose() {
        BaseSubscription.logger.debug("Destroy Subscription of topic : " + this.topicName);

        if (this.ownerNode.getSubscriptions().contains(this)) {
            this.ownerNode.getSubscriptions().remove(this);
        }
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Subscription#getCallback()
     */
    @Override
    public SubscriptionCallback<T> getCallback() {
        return this.callback;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Subscription#getMessageType()
     */
    @Override
    public Class<T> getMessageType() {
        return this.messageType;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Subscription#getNode()
     */
    @Override
    public Node getNode() {
        return this.ownerNode;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Subscription#getQosProfile()
     */
    @Override
    public QoSProfile getQosProfile() {
        return qosProfile;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Subscription#getTopicName()
     */
    @Override
    public String getTopicName() {
        return this.topicName;
    }

}

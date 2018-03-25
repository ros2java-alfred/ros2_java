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

import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.qos.QoSProfile;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 * @param <T>
 */
public abstract class BasePublisher<T extends Message> implements Publisher<T> {

    private static final Logger logger = LoggerFactory.getLogger(BasePublisher.class);

    /** Node owner. */
    private final Node ownerNode;

    /** Message Type. */
    private final Class<T> messageType;

    /** The topic to which this publisher will publish messages. */
    private final String topicName;

    /** Quality of Service profile. */
    private final QoSProfile qosProfile;

    public BasePublisher(final Node node, final Class<T> messageType, final String topic, final QoSProfile qosProfile) {
        if (node == null) {
            throw new RuntimeException("Need to provide active node with handle object");
        }

        this.ownerNode = node;
        this.messageType = messageType;
        this.topicName = topic;
        this.qosProfile = qosProfile;

        BasePublisher.logger.debug("Create Publisher of topic : " + this.topicName);
        this.ownerNode.getPublishers().add(this);
    }

    /* (non-Javadoc)
     * @see java.lang.AutoCloseable#close()
     */
    @Override
    public void close() throws Exception {
        this.dispose();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#dispose()
     */
    @Override
    public void dispose() {
        BasePublisher.logger.debug("Destroy Publisher of topic : " + this.topicName);

        if (this.ownerNode.getPublishers().contains(this)) {
            this.ownerNode.getPublishers().remove(this);
        }
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#doInterProcessPublish(org.ros2.rcljava.internal.message.Message)
     */
    @Override
    public void doInterProcessPublish(final T message) {
        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getGid()
     */
    @Override
    public String getGid() {
        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getIntraProcessGid()
     */
    @Override
    public String getIntraProcessGid() {
        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getMessageType()
     */
    @Override
    public Class<T> getMessageType() {
        return this.messageType;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getNode()
     */
    @Override
    public Node getNode() {
        return this.ownerNode;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getQosProfile()
     */
    @Override
    public QoSProfile getQosProfile() {
        return this.qosProfile;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getQueueSize()
     */
    @Override
    public int getQueueSize() {
        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getTopicName()
     */
    @Override
    public String getTopicName() {
        return this.topicName;
    }
}

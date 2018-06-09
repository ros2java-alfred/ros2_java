/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.NativeNode;
import org.ros2.rcljava.qos.QoSProfile;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * This class serves as a bridge between ROS2's rcl_publisher_t and RCLJava.
 * A Publisher must be created via
 * {@link Node#createPublisher(Class&lt;T&gt;, String)}
 *
 * @param <T> The type of the messages that this publisher will publish.
 */
public class NativePublisher<T extends Message> extends BasePublisher<T> {

    private static final Logger logger = LoggerFactory.getLogger(NativePublisher.class);

    /**
     * An integer that represents a pointer to the underlying ROS2 publisher
     * structure (rcl_publisher_t).
     */
    private final long publisherHandle;

    // Native call.
    private static native <T extends Message> long nativeCreatePublisherHandle(
            long nodeHandle, Class<T> messageType, String topic, long qosProfileHandle);

    /**
     * Publish a message via the underlying ROS2 mechanisms.
     *
     * @param <T> The type of the messages that this publisher will publish.
     * @param publisherHandle A pointer to the underlying ROS2 publisher
     *     structure, as an integer. Must not be zero.
     * @param message An instance of the &lt;T&gt; parameter.
     */
    private static native <T extends Message> void nativePublish(long publisherHandle, T message);

    /**
     * Destroy a ROS2 publisher (rcl_publisher_t).
     *
     * @param nodeHandle A pointer to the underlying ROS2 node structure that
     *     created this publisher, as an integer. Must not be zero.
     * @param publisherHandle A pointer to the underlying ROS2 publisher
     *     structure, as an integer. Must not be zero.
     */
    private static native void nativeDispose(long nodeHandle, long publisherHandle);

    /**
     * Constructor.
     *
     * @param node Node instance associated.
     * @param messageType
     * @param topicName The topic to which this publisher will publish messages.
     * @param qosProfile Quality of Service profile.
     */
    public NativePublisher(
            final NativeNode node,
            final Class<T> messageType,
            final String topicName,
            final QoSProfile qosProfile) {
        super(node, messageType, topicName, qosProfile);

        final String fqnTopic =  GraphName.getFullName(node, topicName, null);
        if (!GraphName.isValidTopic(fqnTopic)) { throw new RuntimeException("Invalid topic name."); }

        final long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
        this.publisherHandle = NativePublisher.nativeCreatePublisherHandle(
                this.getNode().getNodeHandle(),
                messageType,
                fqnTopic,
                qosProfileHandle);
        RCLJava.disposeQoSProfile(qosProfileHandle);
        if (this.publisherHandle == 0) { throw new RuntimeException("Publisher Handle is not define !"); }

        NativePublisher.logger.debug(
                String.format("Created Native Publisher of topic : %s [0x%x]",
                        this.getTopicName(),
                        this.publisherHandle));
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#dispose()
     */
    @Override
    public void dispose() {
        super.dispose();

        NativePublisher.logger.debug(
                String.format("Destroy Native Publisher of topic : %s [0x%x]",
                        this.getTopicName(),
                        this.publisherHandle));

        NativePublisher.nativeDispose(this.getNode().getNodeHandle(), this.publisherHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#getNode()
     */
    @Override
    public NativeNode getNode() {
        return (NativeNode) super.getNode();
    }

    /**
     * @return The pointer to the underlying ROS2 publisher structure.
     */
    public final long getPublisherHandle() {
        return this.publisherHandle;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Publisher#publish(org.ros2.rcljava.internal.message.Message)
     */
    @Override
    public void publish(final T message) {
//        NativePublisher.logger.debug(
//                String.format("Publish on Native Publisher of topic : %s [0x%x]",
//                        this.getTopicName(),
//                        this.publisherHandle));

        NativePublisher.nativePublish(this.publisherHandle, message);
    }

}

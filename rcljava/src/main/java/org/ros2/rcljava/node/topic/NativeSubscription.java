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
 * This class serves as a bridge between ROS2's rcl_subscription_t and RCLJava.
 * A Subscription must be created via
 * {@link Node#createSubscription(Class&lt;T&gt;, String, Consumer&lt;T&gt;)}
 *
 * @param <T> The type of the messages that this subscription will receive.
 */
public class NativeSubscription<T extends Message> extends BaseSubscription<T> {

    private static final Logger logger = LoggerFactory.getLogger(NativeSubscription.class);

    /**
     * An integer that represents a pointer to the underlying ROS2 subscription structure (rcl_subsription_t).
     */
    private final long subscriptionHandle;

    // Native call.
    private static native <T> long nativeCreateSubscriptionHandle(
            long nodeHandle, Class<T> messageType, String topic, long qosProfileHandle);

    /**
     * Destroy a ROS2 subscription (rcl_subscription_t).
     *
     * @param nodeHandle A pointer to the underlying ROS2 node structure that
     *     created this subscription, as an integer. Must not be zero.
     * @param subscriptionHandle A pointer to the underlying ROS2 subscription
     *     structure, as an integer. Must not be zero.
     */
    private static native void nativeDispose(long nodeHandle, long subscriptionHandle);

    /**
     * Constructor.
     *
     * @param node Node structure that created this subscription.
     * @param messageType The <code>Class</code> of the messages that this
     *     subscription will receive. We need this because of Java's type erasure,
     *     which doesn't allow us to use the generic parameter of
     *     {@link org.ros2.rcljava.Subscription} directly.
     * @param topicName The topic to which this subscription will be subscribed.
     * @param callback The callback function that will be triggered when a new
     *     message is received.
     * @param qosProfile Quality of Service profile.
     */
    public NativeSubscription(
            final NativeNode node,
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qosProfile) {
        super(node, messageType, topicName, callback, qosProfile);

        final String fqnTopic =  GraphName.getFullName(node, topicName, null);
        if (!GraphName.isValidTopic(fqnTopic)) { throw new RuntimeException("Invalid topic name."); }

        final long qosProfileHandle = RCLJava.convertQoSProfileToHandle(qosProfile);
        this.subscriptionHandle = NativeSubscription.nativeCreateSubscriptionHandle(
                this.getNode().getNodeHandle(),
                messageType,
                fqnTopic,
                qosProfileHandle);
        RCLJava.disposeQoSProfile(qosProfileHandle);
        if (this.subscriptionHandle == 0) { throw new RuntimeException("Need to provide active subscribtion with handle object"); }

        NativeSubscription.logger.debug(
                String.format("Created Native Subscription of topic : %s [0x%x]",
                        this.getTopicName(),
                        this.subscriptionHandle));
    }

    @Override
    public void dispose() {
        super.dispose();

        NativeSubscription.logger.debug(
                String.format("Destroy Native Subscription of topic : %s [0x%x]",
                        this.getTopicName(),
                        this.subscriptionHandle));

        NativeSubscription.nativeDispose(this.getNode().getNodeHandle(), this.subscriptionHandle);
    }

    @Override
    public final NativeNode getNode() {
        return (NativeNode) super.getNode();
    }

    /**
     * @return The pointer to the underlying ROS2 subscription structure.
     */
    public final long getSubscriptionHandle() {
        return this.subscriptionHandle;
    }
}

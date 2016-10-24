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
package org.ros2.rcljava.node.topic;

import org.ros2.rcljava.QoSProfile;
import org.ros2.rcljava.internal.message.Message;

/**
 * This class serves as a bridge between ROS2's rcl_subscription_t and RCLJava.
 * A Subscription must be created via
 * @{link Node#createSubscription(Class&lt;T&gt;, String, Consumer&lt;T&gt;)}
 *
 * @param <T> The type of the messages that this subscription will receive.
 * @author Esteve Fernandez <esteve@apache.org>
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class Subscription<T extends Message> {

    /**
       * An integer that represents a pointer to the underlying ROS2 node
       * structure (rcl_node_t).
       */
    private final long nodeHandle;

    /**
     * An integer that represents a pointer to the underlying ROS2 subscription
     * structure (rcl_subsription_t).
     */
    private final long subscriptionHandle;

    /**
     * The class of the messages that this subscription may receive.
     */
    private final Class<T> messageType;

    /**
     * The topic to which this subscription is subscribed.
     */
    private final String topic;

    /**
     * The callback function that will be triggered when a new message is
     * received.
     */
    private final Consumer<T> callback;

    /** Quality of Service profil. */
    private final QoSProfile qosProfile;

    // Native call.
    //private static native void nativeDispose(long nodeHandle, long publisherHandle);

    /**
     * Constructor.
     *
     * @param nodeHandle A pointer to the underlying ROS2 node structure that
     *     created this subscription, as an integer. Must not be zero.
     * @param subscriptionHandle A pointer to the underlying ROS2 subscription
     *     structure, as an integer. Must not be zero.
     * @param messageType The <code>Class</code> of the messages that this
     *     subscription will receive. We need this because of Java's type erasure,
     *     which doesn't allow us to use the generic parameter of
     *     @{link org.ros2.rcljava.Subscription} directly.
     * @param topic The topic to which this subscription will be subscribed.
     * @param callback The callback function that will be triggered when a new
     *     message is received.
     */
    public Subscription(final long nodeHandle,final  long subscriptionHandle,final  Class<T> messageType,final  String topic,final  Consumer<T> callback,final  QoSProfile qosProfile) {
        this.nodeHandle = nodeHandle;
        this.subscriptionHandle = subscriptionHandle;
        this.messageType = messageType;
        this.topic = topic;
        this.callback = callback;
        this.qosProfile = qosProfile;
    }

    public final long getNodeHandle() {
        return this.nodeHandle;
      }

    /**
     * @return The callback function that this subscription will trigger when
     *     a message is received.
     */
    public final Consumer<T> getCallback() {
        return this.callback;
    }

    /**
     * @return The pointer to the underlying ROS2 subscription structure.
     */
    public final long getSubscriptionHandle() {
        return this.subscriptionHandle;
    }

    /**
     * @return The type of the messages that this subscription may receive.
     */
    public final Class<T> getMessageType() {
        return this.messageType;
    }

    /**
     * Get topic name.
     * @return
     */
    public final String getTopic() {
        return this.topic;
    }

    /**
     * Release all Publisher ressource.
     */
    public void dispose() {
        // Subscription.nativeDispose(this.nodeHandle, this.subscriptionHandle);
    }
}

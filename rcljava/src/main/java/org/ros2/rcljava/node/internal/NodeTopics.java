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

package org.ros2.rcljava.node.internal;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.qos.QoSProfile;

public interface NodeTopics {

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param messageType Message class.
     * @param topicName The topic for this publisher to publish on.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Publisher instance of the created publisher.
     */
    <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName,
            final QoSProfile qos);

    /**
     * Create and return a Publisher.
     *
     * @param <T> Message definition.
     * @param messageType Message class.
     * @param topicName The topic for this publisher to publish on.
     * @param qosHistoryDepth The depth of the publisher message queue.
     * @return Publisher instance of the created publisher.
     */
    <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName,
            final int qosHistoryDepth);

    /**
     * Create and return a Publisher. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param messageType Message class.
     * @param topicName The topic for this publisher to publish on.
     * @return Publisher instance of the created publisher.
     */
    <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName);

//    /**
//     *
//     * @param publisher
//     */
//    <T extends Message> void addPublisher(final Publisher<T> publisher);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param messageType Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @param ignoreLocalPublications True to ignore local publications.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qos,
            final boolean ignoreLocalPublications);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param messageType Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qos The quality of service profile to pass on to the rmw implementation.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qos);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param messageType Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qosHistoryDepth The depth of the subscription's incoming message queue.
     * @param ignoreLocalPublications True to ignore local publications.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final int qosHistoryDepth,
            final boolean ignoreLocalPublications);

    /**
     * Create and return a Subscription.
     *
     * @param <T> Message definition.
     * @param messageType Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @param qosHistoryDepth The depth of the subscription's incoming message queue.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final int qosHistoryDepth);

    /**
     * Create and return a Subscription. (Retro-compatibility)
     *
     * @param <T> Message definition.
     * @param messageType Message Class
     * @param topicName The topic to subscribe on.
     * @param callback The user-defined callback function.
     * @return Subscription instance of the created subscription.
     */
    <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback);

//    /**
//     *
//     * @param subscription
//     */
//    <T extends Message> void addSubscription(final Subscription<T> subscription);
}

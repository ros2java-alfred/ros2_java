/* Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

public interface Subscription<T extends Message> extends AutoCloseable {

    /**
     * Safely destroy the underlying ROS2 subscriber structure.
     */
    void dispose();

    /**
     * Get CallBack object.
     * @return The callback function that this subscription will trigger when a message is received.
     */
    SubscriptionCallback<T> getCallback();

    /**
     * @return The type of the messages that this subscription may receive.
     */
    Class<T> getMessageType();

    /**
     * @return Return owner Node of the subscription.
     */
    Node getNode();

    /**
     * @return Return QOS Profile of the subscription.
     */
    QoSProfile getQosProfile();

    /**
     * Get the topic that this subscription is subscribed on.
     *
     * @return Return topic name of the subscription.
     */
    String getTopicName();

}

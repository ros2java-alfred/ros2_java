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

public interface Publisher<T extends Message> extends AutoCloseable {

    /**
     * Safely destroy the underlying ROS2 publisher structure.
     */
    void dispose();

    /**
     *
     * @param message An instance of the message to send.
     */
    void doInterProcessPublish(final T message);

    /**
     * Get the global identifier for this publisher (used in rmw and by DDS).
     *
     * @return The gid.
     */
    String getGid();

    /**
     * Get the global identifier for this publisher used by intra-process communication.
     *
     * @return The intra-process gid.
     */
    String getIntraProcessGid();

    /**
     * @return the messageType
     */
    Class<T> getMessageType();

    /**
     *
     * @return
     */
    Node getNode();

    /**
     * @return the qosProfile
     */
    QoSProfile getQosProfile();

    /**
     * Get the queue size for this publisher.
     *
     * @return The queue size.
     */
    int getQueueSize();

    /**
     * Get the topic that this publisher publishes on.
     *
     * @return The topic name.
     */
    String getTopicName();

    /**
     * Send a message to the topic for this publisher.
     * This function is templated on the input message type, Message T.
     *
     * @param message An instance of the message to send.
     */
    void publish(final T message);

}

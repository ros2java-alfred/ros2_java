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
import org.ros2.rcljava.node.JavaNode;
import org.ros2.rcljava.qos.QoSProfile;

/**
 * This class is JVM Subscription of RCLJava.
 * <b>Actually not implemented !!!</b>
 */
public class JavaSubscription<T extends Message> extends BaseSubscription<T> {

    public JavaSubscription(
            final JavaNode node,
            final Class<T> messageType,
            final String topic,
            final SubscriptionCallback<T> callback,
            final QoSProfile qosProfile) {
        super(node, messageType, topic, callback, qosProfile);

        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.topic.Subscription#getNode()
     */
    @Override
    public JavaNode getNode() {
        return (JavaNode) super.getNode();
    }
}

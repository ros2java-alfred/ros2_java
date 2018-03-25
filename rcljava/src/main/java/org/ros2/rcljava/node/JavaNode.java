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

import java.util.List;
import java.util.Map;

import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.qos.QoSProfile;

/**
 * This class is JVM Node of RCLJava.
 * <b>Actually not implemented !!!</b>
 */
public class JavaNode extends BaseNode {

    public JavaNode(final String namespace, final String defaultName, final String... args) {
        super(namespace, defaultName, args);

        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.Node#getTopicNamesAndTypes(boolean)
     */
    @Override
    public Map<String, List<String>> getTopicNamesAndTypes(final boolean noDemangle) {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.internal.NodeServices#createClient(java.lang.Class, java.lang.String, org.ros2.rcljava.qos.QoSProfile)
     */
    @Override
    public <T extends MessageService> Client<T> createClient(
            final Class<T> serviceType,
            final String serviceName,
            final QoSProfile qos) {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.internal.NodeServices#createService(java.lang.Class, java.lang.String, org.ros2.rcljava.node.service.ServiceCallback, org.ros2.rcljava.qos.QoSProfile)
     */
    @Override
    public <T extends MessageService> Service<T> createService(
            final Class<T> serviceType,
            final String serviceName,
            final ServiceCallback<?, ?> callback,
            final QoSProfile qos) {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.internal.NodeTopics#createPublisher(java.lang.Class, java.lang.String, org.ros2.rcljava.qos.QoSProfile)
     */
    @Override
    public <T extends Message> Publisher<T> createPublisher(
            final Class<T> messageType,
            final String topicName,
            final QoSProfile qos) {
        // TODO Auto-generated method stub
        return null;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.node.internal.NodeTopics#createSubscription(java.lang.Class, java.lang.String, org.ros2.rcljava.node.topic.SubscriptionCallback, org.ros2.rcljava.qos.QoSProfile, boolean)
     */
    @Override
    public <T extends Message> Subscription<T> createSubscription(
            final Class<T> messageType,
            final String topicName,
            final SubscriptionCallback<T> callback,
            final QoSProfile qos,
            final boolean ignoreLocalPublications) {
        // TODO Auto-generated method stub
        return null;
    }

}

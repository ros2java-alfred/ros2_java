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

package org.ros2.rcljava.node;

import java.util.List;
import java.util.Map;
import java.util.Queue;

import org.ros2.rcljava.time.WallTimer;

import builtin_interfaces.msg.Time;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;
import org.ros2.rcljava.node.internal.NodeGraph;
import org.ros2.rcljava.node.internal.NodeLogging;
import org.ros2.rcljava.node.internal.NodeParameters;
import org.ros2.rcljava.node.internal.NodeServices;
import org.ros2.rcljava.node.internal.NodeTimers;
import org.ros2.rcljava.node.internal.NodeTopics;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;

/**
 * ROS2 Client API.
 */
public interface Node extends
//        NodeClock,
        NodeGraph,
        NodeLogging,
        NodeParameters,
        NodeServices,
        NodeTimers,
        NodeTopics,
        AutoCloseable {

    /**
     * @return Return the name of the node.
     */
    String getName();

    /**
     * @return Return the namespace of the node.
     */
    String getNameSpace();

//    /**
//     * @return Return Logger Name of the node.
//     */
//    String getLoggerName();
//
//    /**
//     * @return Return the context of the node.
//     */
//    Context getContext();








    /**
     *
     * @return
     */
    Map<String, List<String>> getServiceNamesAndTypes();

    @Deprecated
    Map<String, List<String>> getTopicNamesAndTypes(final boolean noDemangle);

    /**
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     * Return the number of on loan graph events, see get_graph_event().
     */
    int countGraphUsers();

    List<Byte> getParametersTypes(final List<String> names);

    List<String> getParametersNames();

    /**
     * @return All the @{link Subscription}s that were created by this instance.
     */
    Queue<Subscription<? extends Message>> getSubscriptions();

    /**
     * @return All the @{link Publisher}s that were created by this instance.
     */
    Queue<Publisher<? extends Message>> getPublishers();

    /**
     * Get list of Clients.
     * @return ArrayList of Clients
     */
    Queue<Client<? extends MessageService>> getClients();

    /**
     * Get list of Services.
     * @return ArrayList of Services
     */
    Queue<Service<? extends MessageService>> getServices();

    /**
     * @return All the @{link WallTimer}s that were created by this instance.
     */
    Queue<WallTimer> getWallTimers();

    Time getCurrentTime();

    /**
     * Release all resource.
     */
    void dispose();
}

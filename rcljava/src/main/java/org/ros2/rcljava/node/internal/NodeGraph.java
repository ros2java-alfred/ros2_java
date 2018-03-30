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

import java.util.Map;
import java.util.List;

/**
 * Interface of Node Graph stack.
 */
public interface NodeGraph {

    /**
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     * Return the number of on loan graph events, see get_graph_event().
     */
    int countGraphUsers();

    /**
     *
     * @return
     */
    Map<String, List<String>> getServiceNamesAndTypes();

    /**
     * A topic is considered to exist when at least one publisher or subscriber
     * exists for it, whether they be local or remote to this process.
     *
     * @param noDemangle if true, topic names and types are not demangled
     * @return Return a map of existing topic names to list of topic types.
     */
    @Deprecated
    Map<String, List<String>> getTopicNamesAndTypes(final boolean noDemangle);

    /**
     * A service is considered to exist when at least one service server or
     * service client exists for it, whether they be local or remote to this
     * process.
     *
     * @return Return a map of existing service names to list of service types.
     */
    Map<String, List<String>> getTopicNamesAndTypes();

    /**
     * @return Return a Array of existing node names (string).
     */
    List<String> getNodeNames();

    /**
     * @param topic
     * @return Return the number of publishers that are advertised on a given topic.
     */
    int countPublishers(final String topic);

    /**
     * @param topic
     * @return Return the number of subscribers who have created a subscription for a given topic.
     */
    int countSubscribers(final String topic);

//    /**
//     *
//     * @return Return the rcl guard condition which is triggered when the ROS graph changes.
//     */
//    Object getGraphGuardCondition();

    /**
     * Notify threads waiting on graph changes.
     *
     * Affects threads waiting on the notify guard condition, see:
     * get_notify_guard_condition(), as well as the threads waiting on graph
     * changes using a graph Event, see: wait_for_graph_change().
     *
     * This is typically only used by the rclcpp::graph_listener::GraphListener.
     *
     * @return Return a graph event, which will be set anytime a graph change occurs.
     * @throws RCLBaseError (a child of that exception) when an rcl error occurs
     */
    void notifyGraphChange();

    /** Notify any and all blocking node actions that shutdown has occurred. */
    void notifyShutdown();

    /**
     * The graph Event object is a loan which must be returned.
     * The Event object is scoped and therefore to return the load just let it go
     * out of scope.
     *
     * @return Return a graph event, which will be set anytime a graph change occurs.
     */
    Object getGraphEvent();

    /**
     * Wait for a graph event to occur by waiting on an Event to become set.
     *
     * The given Event must be acquire through the get_graph_event() method.
     *
     * @throws InvalidEventError if the given event is nullptr
     * @throws EventNotRegisteredError if the given event was not acquired with
     *   get_graph_event().
     */
    void waitForGraphChange(final Object event, final int timeout);

}

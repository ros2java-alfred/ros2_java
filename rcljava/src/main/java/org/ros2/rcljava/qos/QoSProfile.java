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

package org.ros2.rcljava.qos;

import org.ros2.rcljava.qos.policies.Durability;
import org.ros2.rcljava.qos.policies.History;
import org.ros2.rcljava.qos.policies.Reliability;

/**
 * Quality of Service profile.
 */
public class QoSProfile {

    public static final int DEPTH_SYSTEM_DEFAULT = 0;

    public static final QoSProfile SENSOR_DATA = new QoSProfile(
            History.KEEP_LAST,
            5,
            Reliability.BEST_EFFORT,
            Durability.VOLATILE);

    /**
     * Default for Parameter layer.
     */
    public static final QoSProfile PARAMETER = new QoSProfile(
            History.KEEP_LAST,
            1000,
            Reliability.RELIABLE,
            Durability.VOLATILE);

    /**
     * Default for Sub/Pub layer.
     */
    public static final QoSProfile DEFAULT = new QoSProfile(
            History.KEEP_LAST,
            10,
            Reliability.RELIABLE,
            Durability.VOLATILE);

    /**
     * Default for Service layer.
     */
    public static final QoSProfile SERVICES_DEFAULT = new QoSProfile(
            History.KEEP_LAST,
            10,
            Reliability.RELIABLE,
            Durability.VOLATILE);

    public static final QoSProfile PARAMETER_EVENTS = new QoSProfile(
            History.KEEP_ALL,
            1000,
            Reliability.RELIABLE,
            Durability.VOLATILE);

    public static final QoSProfile SYSTEM_DEFAULT = new QoSProfile(
            History.SYSTEM_DEFAULT,
            DEPTH_SYSTEM_DEFAULT,
            Reliability.SYSTEM_DEFAULT,
            Durability.SYSTEM_DEFAULT);

    /**
     * History mode.
     */
    private final History history;

    /**
     * Depth.
     */
    private final int depth;

    /**
     * Reliability.
     */
    private final Reliability reliability;

    /**
     * Durability.
     */
    private final Durability durability;

    /**
     * If true, any ROS specific namespacing conventions will be circumvented.
     *
     * In the case of DDS and topics, for example, this means the typical
     * ROS specific prefix of `rt` would not be applied as described here:
     *
     *   {@linkplain http://design.ros2.org/articles/topic_and_service_names.html#ros-specific-namespace-prefix}
     *
     * This might be useful when trying to directly connect a native DDS topic
     * with a ROS 2 topic.
     */
    private final boolean avoidRosNamespaceConventions = false;

    /**
     * Constructor.
     * @param history History
     * @param depth Depth
     * @param reliability Reliability
     * @param durability Durability
     */
    public QoSProfile(final History history, final int depth, final Reliability reliability, final Durability durability) {
        this.history = history;
        this.depth = depth;
        this.reliability = reliability;
        this.durability = durability;
    }

    public final History getHistory() {
        return this.history;
    }

    public final int getDepth() {
        return this.depth;
    }

    public final Reliability getReliability() {
        return this.reliability;
    }

    public final Durability getDurability() {
        return this.durability;
    }

    public final boolean getAvoidRosNamespaceConventions() {
        return this.avoidRosNamespaceConventions;
    }
}

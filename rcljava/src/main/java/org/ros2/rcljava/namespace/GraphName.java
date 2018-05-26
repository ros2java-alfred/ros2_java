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

package org.ros2.rcljava.namespace;

import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.topic.Topics;

/**
 * Graph-name API.
 */
public final class GraphName {

    private static final String EMPTY = "";
    private static final String PRIVATE = "_";
    private static final String SEPARATOR = "/";
    private static final String RELATIVE = "~";
    private static final String PARAMETER = "{}";
    private static final String RELATIVE_TOPIC = RELATIVE + SEPARATOR;
    private static final String FORMAT_TOPIC = "%s" + SEPARATOR + "%s";

    /**
     * All the @{link Node}s that have been created.
     */
    private static Queue<Node> nodes = new LinkedBlockingQueue<Node>();

    private GraphName() { super(); }

    /**
     * Add Node to Graph API.
     * @param node to add to Graph.
     */
    public static void addNode(final Node node) {
//        if (initialized) {
        GraphName.nodes.add(node);
//        }
    }

    /**
     * Remove Node to Graph API.
     * @param node to remove to Graph.
     */
    public static void removeNode(final Node node) {
//        if (initialized) {
        GraphName.nodes.remove(node);
//        }
    }

    /**
     * Release Graph API.
     */
    public static void dispose() {
        for (final Node node : GraphName.nodes) {
            node.dispose();
        }
    }

    private static String removeSheme(final String url) {
        String topicName = url;
        final boolean result = topicName != null;

        if (result) {
            topicName = topicName.replaceAll(Topics.SCHEME, EMPTY);
            topicName = topicName.replaceAll(Service.SCHEME, EMPTY);
        }

        return topicName;
    }

    /**
     * Make a valid full-name for ROS.
     *
     * @param nameSpace
     * @param nodeName
     * @return Generate Full name.
     */
    public static String getFullName(final String nameSpace, final String nodeName) {
        String result = nodeName;
        String ns = nameSpace;

        // Manage Name Space.
        if (ns == null || ns.length() == 0) {
            ns = EMPTY;
        } else if (!ns.startsWith(SEPARATOR)){
            ns = String.format(FORMAT_TOPIC, EMPTY, ns);
        }

        // Absolute case
        if (nodeName.startsWith(SEPARATOR)) {
            result = nodeName;
        } else

        {
            result = String.format(FORMAT_TOPIC, ns, nodeName);
        }

        return result;
    }
    /**
     *
     *
     * <table>
     * <tr><td>Input Name	</td><td>Node: my_node NS: none	</td><td>Node: my_node NS: /my_ns</td></tr>
     * <tr><td>ping			</td><td>/ping					</td><td>/my_ns/ping</td></tr>
     * <tr><td>/ping		</td><td>/ping					</td><td>/ping</td></tr>
     * <tr><td>~			</td><td>/my_node				</td><td>/my_ns/my_node</td></tr>
     * <tr><td>~/ping		</td><td>/my_node/ping			</td><td>/my_ns/my_node/ping</td></tr></table>
     *
     * @param node
     * @param name
     * @param options
     * @return Full path of node.
     */
    public static String getFullName(final Node node, final String name, final List<String> options) {
        String result = name;
        String nodeName = node.getName();
        String ns = node.getNameSpace();

        // Manage Name Space.
        if (ns == null || ns.length() == 0) {
            ns = EMPTY;
        } else if (!ns.startsWith(SEPARATOR)){
            ns = String.format(FORMAT_TOPIC, EMPTY, ns);
        }

        if (nodeName.startsWith(SEPARATOR)) {
            nodeName = nodeName.replaceFirst(SEPARATOR, EMPTY);
        }

        // Absolute case
        if (name.startsWith(SEPARATOR)) {
            result = name;
        } else

        // Relative case
        if (name.startsWith(RELATIVE_TOPIC)) {
            final String relPath = name.replaceFirst(RELATIVE_TOPIC, EMPTY);
            result = String.format(FORMAT_TOPIC, String.format(FORMAT_TOPIC, ns, nodeName), relPath);
        } else

        // Node case
        if (name.startsWith(RELATIVE)) {
            result = String.format(FORMAT_TOPIC, ns, nodeName);
        } else

        // Name space path case
        {
            result = String.format(FORMAT_TOPIC, ns, name);
        }

        return result;
    }

    /**
     * Check if name is valid.
     *
     * For testing : https://regex101.com/r/b3R65h/1
     * @param name Topic name.
     * @return true if valid topic.
     */
    public static boolean isValidTopic(final String name) {
        // Delete sample Regex : https://regex101.com/delete/7qMZiThwlEla6eAKIIee3251
        boolean result = name != null;

        if (result) {
            final String topicName = GraphName.removeSheme(name);

            result =
                    topicName != null &&
                    topicName.length() > 0 &&
                    topicName.matches("^(?!.*(//|__|_/))[~A-Za-z/_{}][_A-Za-z0-9/{}]*$") &&
                    !topicName.endsWith(PRIVATE) &&
                    !topicName.endsWith(SEPARATOR)
            ;

            if (result && topicName.startsWith(RELATIVE) && topicName.length() > 1) {
                result = topicName.startsWith(RELATIVE_TOPIC);
            }
        }

        return result;
    }

    /**
     * Check if substitution is valid.
     *
     * @param substitution Substitution string.
     * @return true if substitution is valid.
     */
    public static boolean isValidSubstitution(final String substitution) {
        return substitution != null &&
               substitution.length() > 0 &&
               substitution.matches("^[_A-Za-z_/][_A-Za-z0-9/]*$") &&
               !substitution.endsWith(PRIVATE);
    }

    /**
     * Chack if Full Qualify Name is valid.
     *
     * @param fqn
     * @return true if Full Qualified Domain Name is valid.
     */
    public static boolean isValidFQDN(final String fqn) {
        boolean result = fqn != null;

        if (result) {
            final String topicName = GraphName.removeSheme(fqn);

            result = GraphName.isValidTopic(topicName) &&
                     topicName.startsWith(SEPARATOR) &&
                     !topicName.contains(RELATIVE) &&
                     !topicName.contains(PARAMETER);
        }

        return result;
    }
}

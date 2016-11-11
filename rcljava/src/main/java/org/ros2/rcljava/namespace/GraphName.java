/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016 Mickael Gaillard <mick.gaillard@gmail.com>
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

import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import org.ros2.rcljava.internal.IGraph;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.topic.Topics;

public class GraphName implements IGraph {

    /**
     * All the @{link Node}s that have been created.
     */
    private static Queue<Node> nodes = new LinkedBlockingQueue<Node>();

    public static void addNode(Node node) {
//        if (initialized) {
        GraphName.nodes.add(node);
//        }
    }

    public static void removeNode(Node node) {
//        if (initialized) {
        GraphName.nodes.remove(node);
//        }
    }

    public static void dispose() {
        for (Node node : GraphName.nodes) {
            node.dispose();
        }
    }

    private static String removeSheme(final String url) {
        String topicName = url;
        boolean result = topicName != null;

        if (result) {
            topicName = topicName.replaceAll(Topics.SCHEME, "");
            topicName = topicName.replaceAll(Service.SCHEME, "");
        }

        return topicName;
    }

    /**
     * Check if name is valid.
     *
     * For testing : https://regex101.com/r/b3R65h/1
     * @param topicName
     * @return
     */
    public static boolean isValidTopic(final String name) {
        // Delete sample Regex : https://regex101.com/delete/7qMZiThwlEla6eAKIIee3251
        boolean result = name != null;

        if (result) {
            String topicName = removeSheme(name);

            result =
                    topicName != null &&
                    topicName.length() > 0 &&
                    topicName.matches("^(?!.*(//|__|_/))[~A-Za-z/_{}][_A-Za-z0-9/{}]*$") &&
                    !topicName.endsWith("_") &&
                    !topicName.endsWith("/")
            ;

            if (result && topicName.startsWith("~") && topicName.length() > 2) {
                result = topicName.startsWith("~/");
            }
        }

        return result;
    }

    /**
     *
     * @param substitution
     * @return
     */
    public static boolean isValidSubstitution(final String substitution) {
        boolean result =
                substitution != null &&
                substitution.length() > 0 &&
                substitution.matches("^[_A-Za-z_/][_A-Za-z0-9/]*$") &&
                !substitution.endsWith("_");

        return result;
    }

    public static boolean isValideFQDN(final String fqn) {
        boolean result = fqn != null;

        if (result) {
            String topicName = removeSheme(fqn);

            result =
                    GraphName.isValidTopic(topicName) &&
                    topicName.startsWith("/") &&
                    !topicName.contains("~") &&
                    !topicName.contains("{}");
        }

        return result;
    }
}

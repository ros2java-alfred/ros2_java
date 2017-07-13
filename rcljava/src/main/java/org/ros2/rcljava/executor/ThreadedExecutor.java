/* Copyright 2017 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.executor;

import org.ros2.rcljava.node.Node;

/**
 * Executor provides spin functions (including {@link spinNodeOnce} and {@link spinSome}).
 * It coordinates the nodes and callback groups by looking for available work and completing it,
 * based on the threading or concurrency scheme provided by the subclass implementation.
 * An example of available work is executing a subscription callback, or a timer callback.
 * The executor structure allows for a decoupling of the communication graph and the execution
 * model.
 * See {@link SingleThreadedExecutor} and {@link MultiThreadedExecutor} for examples of execution paradigms.
 *
 */
public interface ThreadedExecutor extends Runnable {

    /**
     * Add a node to the executor.
     * @param node to be added.
     */
    void addNode(Node node);

    /**
     * Add a node to the executor.
     * @param node to be added.
     * @param notify True to trigger the interrupt guard condition during this function. If
     * the executor is blocked at the rmw layer while waiting for work and it is notified that a new
     * node was added, it will wake up.
     */
    void addNode(Node node, boolean notify);

    /**
     * Remove a node from the executor.
     * @param node to remove.
     */
    void removeNode(Node node);

    /**
     * Remove a node from the executor.
     * @param node to remove.
     * @param notify True to trigger the interrupt guard condition and wake up the executor.
     * This is useful if the last node was removed from the executor while the executor was blocked
     * waiting for work in another thread, because otherwise the executor would never be notified.
     */
    void removeNode(Node node, boolean notify);

    /**
     * Do work periodically as it becomes available to us. Blocking call, may block indefinitely.
     * It is up to the implementation of Executor to implement spin.
     */
    void spin();

    /**
     * @param timeout
     */
    void spinOnce(long timeout);

    /**
     * Complete all available queued work with blocking.
     * This function can be overridden. The default implementation is suitable for a
     * single-threaded model of execution.
     * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
     * to block (which may have unintended consequences).
     */
    void spinSome();

    /**
     * Add a node to executor, execute the next available unit of work, and remove the node.
     * @param node to add.
     * @param timeout How long to wait for work to become available. Negative values cause
     * spin_node_once to block indefinitely (the default behavior). A timeout of 0 causes this
     * function to be non-blocking.
     */
    void spinNodeOnce(Node node, long timeout);

    /**
     * Add a node, complete all immediately available work, and remove the node.
     * @param node to add.
     */
    void spinNodeSome(Node node);

    void cancel();

}

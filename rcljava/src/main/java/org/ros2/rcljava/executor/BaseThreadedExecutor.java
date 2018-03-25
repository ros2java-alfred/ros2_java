/* Copyright 2017-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ExecutorService;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;

public abstract class BaseThreadedExecutor implements ThreadedExecutor {

    protected final Object mutex = new Object();
    protected final NativeExecutor baseExecutor;
    protected volatile ExecutorService executorService;

    protected final BlockingQueue<Node> nodes = new LinkedBlockingQueue<Node>();

    public BaseThreadedExecutor() {
        this.baseExecutor = new NativeExecutor(this);
    }

    @Override
    public void addNode(Node node) {
        this.addNode(node, false);
    }

    @Override
    public void addNode(Node node, boolean notify) {
        if (!this.nodes.contains(node)) {
            this.nodes.add(node);

//            if (notify) {
//
//            }
        }
    }

    @Override
    public void removeNode(Node node) {
        this.removeNode(node, false);
    }

    @Override
    public void removeNode(Node node, boolean notify) {
        if (this.nodes.contains(node)) {
            this.nodes.remove(node);

            if (notify) {

            }
        }
    }

    @Override
    public void spinSome() {
        AnyExecutable anyExecutable = this.baseExecutor.getNextExecutable();
        while (RCLJava.ok() && anyExecutable != null) {
            BaseThreadedExecutor.executeAnyExecutable(anyExecutable);
            anyExecutable = this.baseExecutor.getNextExecutable(0);
        }
    }

    @Override
    public void spinOnce(long timeout) {
        final AnyExecutable anyExecutable = this.baseExecutor.getNextExecutable(timeout);

        if (anyExecutable != null) {
            BaseThreadedExecutor.executeAnyExecutable(anyExecutable);
        }
    }

    private static void executeAnyExecutable(final AnyExecutable anyExecutable) {
        try {
            NativeExecutor.executeAnyExecutable(anyExecutable);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void spinNodeOnce(Node node, long timeout) {
        this.addNode(node, false);
        this.spinOnce(timeout);
        this.removeNode(node, false);
    }

    @Override
    public void spinNodeSome(Node node) {
        this.addNode(node, false);
        this.spinSome();
        this.removeNode(node, false);
    }

    @Override
    public void run() {
        while (RCLJava.ok()) {
            this.spinOnce(0);
        }
    }

    @Override
    public void cancel() {
        if (!this.executorService.isShutdown()) {
            this.executorService.shutdownNow();
        }
    }

}

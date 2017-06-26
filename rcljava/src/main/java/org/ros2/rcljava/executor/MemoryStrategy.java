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

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.time.WallTimer;

public class MemoryStrategy {

    public ConcurrentLinkedQueue<Subscription<?>> subscriptionHandles = new ConcurrentLinkedQueue<Subscription<?>>();
    public ConcurrentLinkedQueue<WallTimer> timerHandles = new ConcurrentLinkedQueue<WallTimer>();
    public ConcurrentLinkedQueue<Service<?>> serviceHandles = new ConcurrentLinkedQueue<Service<?>>();
    public ConcurrentLinkedQueue<Client<?>> clientHandles = new ConcurrentLinkedQueue<Client<?>>();

    public MemoryStrategy() {

    }

    public void clearHandles() {
        this.subscriptionHandles.clear();
        this.timerHandles.clear();
        this.serviceHandles.clear();
        this.clientHandles.clear();
    }


    public boolean collectEntities(final BlockingQueue<Node> nodes) {
        for (Node node : nodes) {
            for (Subscription<?> subscription : node.getSubscriptions()) {
                this.subscriptionHandles.add(subscription);
            }

            for (Service<?> service : node.getServices()) {
                this.serviceHandles.add(service);
            }

            for (Client<?> client : node.getClients()) {
                this.clientHandles.add(client);
            }

            for (WallTimer timer : node.getWallTimers()) {
                this.timerHandles.add(timer);
            }
        }

        return false;
    }

    /**
     * Provide a newly initialized AnyExecutable object.
     * @return fresh executable.
     */
    public AnyExecutable instantiateNextExecutable() {
        return new AnyExecutable();
    }

    public void getNextSubscription(AnyExecutable anyExecutable, BlockingQueue<Node> nodes) {
        Subscription<?> subscription = this.subscriptionHandles.poll();
        if (subscription != null) {
            anyExecutable.subscription = subscription;
        }
    }

    public void getNextService(AnyExecutable anyExecutable, BlockingQueue<Node> nodes) {
        Service<?> service = this.serviceHandles.poll();
        if (service != null) {
            anyExecutable.service = service;
        }
    }

    public void getNextClient(AnyExecutable anyExecutable, BlockingQueue<Node> nodes) {
        Client<?> client = this.clientHandles.poll();
        if (client != null) {
            anyExecutable.client = client;
        }
    }

    public int numberOfReadySubscriptions() {
        return this.subscriptionHandles.size();
    }

    public int numberOfReadyServices() {
        return this.serviceHandles.size();
    }

    public int numberOfReadyClients() {
        return this.clientHandles.size();
    }

    public int numberOfReadyTimers() {
        return this.timerHandles.size();
    }
//    public int number_of_guard_conditions() {
//        return -1;
//    }

}

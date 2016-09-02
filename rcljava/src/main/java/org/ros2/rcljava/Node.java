/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
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
package org.ros2.rcljava;

import java.lang.ref.WeakReference;

import java.util.ArrayList;
import java.util.List;

public class Node {
   static {
        try {
            System.loadLibrary("rcljavaNode__" + RCLJava.getRMWIdentifier());
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.exit(1);
        }
    }

    private long nodeHandle;
    private List<Subscription> subscriptions = new ArrayList<Subscription>();

    public Node(long nodeHandle) {
        this.nodeHandle = nodeHandle;
    }

    private static native <T> long nativeCreatePublisherHandle(
        long nodeHandle, Class<T> cls, String topic);

    private static native <T> long nativeCreateSubscriptionHandle(
        long nodeHandle, Class<T> cls, String topic);

    public <T> Publisher<T> createPublisher(Class<T> cls, String topic) {
        long publisherHandle = nativeCreatePublisherHandle(this.nodeHandle, cls, topic);
        Publisher<T> publisher = new Publisher<T>(this.nodeHandle, publisherHandle);
        RCLJava.publisherReferences.add(new WeakReference<Publisher>(publisher));
        return publisher;
    }

    public <T> Subscription<T> createSubscription(Class<T> cls, String topic, Consumer<T> callback) {
        long subscriptionHandle = nativeCreateSubscriptionHandle(this.nodeHandle, cls, topic);

        Subscription<T> subscription = new Subscription<T>(this.nodeHandle, subscriptionHandle, cls, topic, callback);
        this.subscriptions.add(subscription);
        return subscription;
    }

    public List<Subscription> getSubscriptions() {
        return this.subscriptions;
    }
}

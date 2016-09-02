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

public class Subscription<T> {

    private long nodeHandle;
    private long subscriptionHandle;
    private Class<T> msgType;
    private String topic;
    private Consumer<T> callback;

    public Subscription(long nodeHandle, long subscriptionHandle, Class<T> msgType, String topic, Consumer<T> callback) {
        this.nodeHandle = nodeHandle;
        this.subscriptionHandle = subscriptionHandle;
        this.msgType = msgType;
        this.topic = topic;
        this.callback = callback;
    }

    public Consumer<T> getCallback() {
        return callback;
    }

    public Class<T> getMsgType() {
        return msgType;
    }

    public long getSubscriptionHandle() {
        return subscriptionHandle;
    }
}

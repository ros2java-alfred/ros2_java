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

package org.ros2.rcljava.node.service;

import java.lang.ref.WeakReference;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executor.ThreadedExecutor;
import org.ros2.rcljava.node.Node;

public class RCLFuture<V> implements Future<V> {

    private ThreadedExecutor executor;
    private WeakReference<Node> nodeReference;
    private boolean done;
    private V value;
    private final Object lock = new Object();

    public RCLFuture(final WeakReference<Node> nodeReference) {
        this.nodeReference = nodeReference;
    }

    public RCLFuture(final ThreadedExecutor executor) {
        this.executor = executor;
    }

    @Override
    public final V get() throws InterruptedException, ExecutionException {
        V result = null;

        if(this.value == null) {
            while (RCLJava.ok() && !this.isDone()) {
                if (this.executor != null) {
                    this.executor.spinOnce(0);
                } else {
                    final Node node = nodeReference.get();
                    if (node == null) {
                        break;
                    } else {
                        result = this.getValue();
                    }
                    RCLJava.spinOnce(node);
                }
            }
        } else {
            result = this.getValue();
        }

        return result;
    }

    @Override
    public final V get(final long timeout, final TimeUnit unit)
            throws InterruptedException, ExecutionException, TimeoutException {
        if (this.isDone()) {
            return this.value;
        }

        long endTime = TimeUnit.NANOSECONDS.convert(System.currentTimeMillis(), TimeUnit.MILLISECONDS);
        final long timeoutNS = TimeUnit.NANOSECONDS.convert(timeout, unit);

        if (timeoutNS > 0) {
            endTime += timeoutNS;
        }

        while (RCLJava.ok()) {
            if (this.executor != null) {
                this.executor.spinOnce(0);
            } else {
                final Node node = nodeReference.get();
                if (node == null) {
                    return null; // TODO(esteve) do something
                }

                RCLJava.spinOnce(node);
            }

            if (this.isDone()) {
                return this.value;
            }

            final long now = TimeUnit.NANOSECONDS.convert(System.currentTimeMillis(), TimeUnit.MILLISECONDS);
            if (now >= endTime) {
                throw new TimeoutException();
            }
        }
        throw new InterruptedException();
    }

    @Override
    public final boolean isDone() {
        return this.done;
    }

    @Override
    public final boolean isCancelled() {
        return false;
    }

    @Override
    public final boolean cancel(final boolean mayInterruptIfRunning) {
        return false;
    }

    public final void set(final V value) {
        synchronized(this.lock) {
            this.value = value;
            this.done = true;
        }
    }

    private V getValue() {
        synchronized(this.lock) {
            return this.value;
        }
    }
}

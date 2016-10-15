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
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;


public class RCLFuture<V> implements Future<V> {
  private WeakReference<Node> nodeReference;
  private boolean done = false;
  private V value = null;

  public RCLFuture(final WeakReference<Node> nodeReference) {
    this.nodeReference = nodeReference;
  }

  public final V get() throws InterruptedException, ExecutionException {
    while (RCLJava.ok() && !isDone()) {
      Node node = nodeReference.get();
      if (node != null) {
        RCLJava.spinOnce(node);
      } else {
        return null; // TODO(esteve) do something
      }
    }
    return value;
  }

  public final V get(final long timeout, final TimeUnit unit) throws
      InterruptedException, ExecutionException, TimeoutException {
    return null;
  }

  public final boolean isDone() {
    return done;
  }

  public final boolean isCancelled() {
    return false;
  }

  public final boolean cancel(final boolean mayInterruptIfRunning) {
    return false;
  }

  public final synchronized void set(final V value) {
    this.value = value;
    done = true;
  }
}

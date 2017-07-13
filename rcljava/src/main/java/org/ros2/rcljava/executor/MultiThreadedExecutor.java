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

import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * Pool-multiple-threaded executor implementation
 * This is the default executor created by {@link RCLJava::spin}.
 */
public final class MultiThreadedExecutor extends BaseThreadedExecutor {

    private int numberOfThreads;

    /**
     * Default constructor. See the default constructor for Executor.
     */
    public MultiThreadedExecutor() {
        this(Runtime.getRuntime().availableProcessors());
    }

    /**
     * Parameters constructor. See the default constructor for Executor.
     */
    public MultiThreadedExecutor(final int numberOfThreads) {
        if (numberOfThreads == 0) {
            this.numberOfThreads = 1;
        } else {
            this.numberOfThreads = numberOfThreads;
        }

        this.executorService = Executors.newFixedThreadPool(this.numberOfThreads);
    }

    /**
     * Pool-multiple threaded implementation of spin.
     * This function will block until work comes in, execute it, and keep blocking.
     * It will only be interrupt by a CTRL-C (managed by the global signal handler).
     * @throws InterruptedException
     */
    @Override
    public void spin() {
        synchronized (mutex) {
            for (int i = 0; i < this.numberOfThreads; i++) {
                 this.executorService.submit(this);
            }
        }

        if (!this.executorService.isShutdown()) {
            this.executorService.shutdown();
            try {
                this.executorService.awaitTermination(2, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public int getNumberOfThreads() {
        return this.numberOfThreads;
    }

}

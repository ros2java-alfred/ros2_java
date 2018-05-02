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

import java.util.concurrent.Executors;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Single-threaded executor implementation
 * This is the default executor created by {@link RCLJava::spin}.
 */
public final class SingleThreadedExecutor extends BaseThreadedExecutor {

    private static final Logger logger = LoggerFactory.getLogger(SingleThreadedExecutor.class);

    /**
     * Default constructor. See the default constructor for Executor.
     */
    public SingleThreadedExecutor() {
        super();

        logger.debug("Initialized Executor.");

        this.executorService = Executors.newSingleThreadExecutor();
    }

    /**
     * Single-threaded implementation of spin.
     * This function will block until work comes in, execute it, and keep blocking.
     * It will only be interrupt by a CTRL-C (managed by the global signal handler).
     */
    @Override
    public void spin() {
        synchronized (mutex) {
            this.executorService.submit(this);
        }

        if (!this.executorService.isShutdown()) {
            this.executorService.shutdown();
        }
    }

}

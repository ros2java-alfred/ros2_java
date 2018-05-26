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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Default Utility of ThreadedExecutor.
 */
public final class DefaultThreadedExecutor {

    private static final Logger logger = LoggerFactory.getLogger(DefaultThreadedExecutor.class);

    private static SingleThreadedExecutor instance;

    public static ThreadedExecutor getInstance() {
        if (instance == null) {
            synchronized (SingleThreadedExecutor.class) {
                if (instance == null) {
                    logger.debug("Create Default Executor.");
                    instance = new SingleThreadedExecutor();
                }
            }
        }
        return instance;
    }

    public static boolean hasInstance() {
        return (instance != null);
    }

    @SuppressWarnings("PMD.NullAssignment")
    public static void dispose() {
        logger.debug("Stop & Destroy Default Executor.");

        instance.clear();
        instance.cancel();
        instance = null;
    }

    private DefaultThreadedExecutor() {
        super();
    }
}

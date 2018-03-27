/* Copyright 2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.node.internal;

import org.ros2.rcljava.Logger;

/**
 * Interface of Node Logging stack.
 */
public interface NodeLogging {

    /**
     * @return Return the logger of the node.
     */
    Logger getLogger();

    /**
     * @return Return the logger name associated with the node.
     */
    String getLoggerName();
}

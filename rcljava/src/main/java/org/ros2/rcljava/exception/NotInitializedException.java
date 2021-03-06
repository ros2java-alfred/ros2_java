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

package org.ros2.rcljava.exception;

/**
 * Raised when the rcljava implementation is accessed before RclJava().
 */
public class NotInitializedException extends RuntimeException {

    /** Serial ID. */
    private static final long serialVersionUID = -5109722435632105485L;

    /**
     * Constructor.
     */
    public NotInitializedException() {
        this("RCLJava.rclJavaInit() has not been called !", null);
    }

    /**
     * Constructor.
     *
     * @param msg cause message.
     */
    public NotInitializedException(final String msg) {
        this(msg, null);
    }

    /**
     * Constructor.
     *
     * @param msg cause message.
     * @param cause Throwable instance of the cause.
     */
    public NotInitializedException(final String msg, final Throwable cause) {
        super(msg, cause);
    }
}

/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016-2017 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.common;

import java.lang.reflect.Method;

/**
 * Utility class that allows generated messages to retrieve the typesupport and
 * the RMW implementation via reflection.
 * This prevents a circular dependency between generated messages and rcljava.
 */
public final class RCLJavaProxy {

    private final static Object lock = new Object();

    /**
     * Private constructor so this cannot be instantiated.
     */
    private RCLJavaProxy() { }

    /**
       * @return a pointer to the underlying typesupport via reflection.
       */
    public static String getTypesupportIdentifier() {
        String result = null;
        synchronized (lock) {
            try {
                final Class<?> cls = Class.forName("org.ros2.rcljava.RCLJava");
                final Method meth = cls.getDeclaredMethod("getTypesupportIdentifier", (Class<?> []) null);
                result = (String)meth.invoke(null);
            } catch(Exception e) {
                // Just return null if we can't find the typesupport identifier
            }
        }
        return result;
    }

    /**
     * @return a pointer to the underlying typesupport via reflection.
     */
    public static String getRMWIdentifier() {
        String result = null;
        synchronized (lock) {
            try {
                final Class<?> cls = Class.forName("org.ros2.rcljava.RCLJava");
                final Method meth = cls.getDeclaredMethod("getRMWIdentifier", (Class<?> []) null);
                result = (String) meth.invoke(null);
            } catch(Exception e) {
                // Just return null if we can't find the RMW identifier
            }
        }
        return result;
    }

    /**
     * @return a pointer to the underlying load native library via reflection.
     */
    public static void loadLibrary(final String name) {
        synchronized (lock) {
            try {
                final Class<?> cls = Class.forName("org.ros2.rcljava.RCLJava");
                final Method meth = cls.getDeclaredMethod("loadLibrary", String.class);
                meth.invoke(null, name);
            } catch(Exception e) {
                // TODO(esteve): handle exception
            }
        }
    }
}

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

package org.ros2.rcljava.node.service;

import java.lang.reflect.Method;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.internal.service.MessageService;

public class NativeServiceType<T extends MessageService> {

    private Class<? extends Message> requestType;

    private long requestFromJavaConverterHandle;
    private long requestToJavaConverterHandle;

    @SuppressWarnings({ "unchecked", "PMD.EmptyCatchBlock" })
    public NativeServiceType(final Class<T> serviceType, final String directionType) {
        try {
            this.requestType = (Class<? extends Message>)serviceType.getField(directionType).get(null);

            final Method requestFromJavaConverterMethod = requestType.getDeclaredMethod("getFromJavaConverter", (Class<?> []) null);
            this.requestFromJavaConverterHandle = (Long)requestFromJavaConverterMethod.invoke(null, (Object []) null);

            final Method requestToJavaConverterMethod = requestType.getDeclaredMethod("getToJavaConverter", (Class<?> []) null);
            this.requestToJavaConverterHandle = (Long)requestToJavaConverterMethod.invoke(null, (Object []) null);
        } catch (Exception e) {
            // simply ignore
        }
    }

    public Class<? extends Message> getType() {
        return this.requestType;
    }

    public Long getFromJavaConverterHandle() {
        return this.requestFromJavaConverterHandle;
    }

    public Long getToJavaConverterHandle() {
        return this.requestToJavaConverterHandle;
    }

}

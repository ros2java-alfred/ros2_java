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

public class Publisher<T> {
   static {
        try {
            System.loadLibrary("rcljavaPublisher__" + RCLJava.getRMWIdentifier());
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.exit(1);
        }
    }

    private long nodeHandle;
    private long publisherHandle;

    public Publisher(long nodeHandle, long publisherHandle) {
        this.nodeHandle = nodeHandle;
        this.publisherHandle = publisherHandle;
    }

    private static native <T> void nativePublish(long publisherHandle, T msg);

    public void publish(T msg) {
        nativePublish(this.publisherHandle, msg);
    }

    private static native void nativeDispose(
        long nodeHandle, long publisherHandle);

    public void dispose() {
        nativeDispose(this.nodeHandle, this.publisherHandle);
    }
}

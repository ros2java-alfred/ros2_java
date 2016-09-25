/* Copyright 2016 Open Source Robotics Foundation, Inc.
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

import org.ros2.rcljava.exception.NotImplementedException;

/**
 * Service Client.
 *
 * @param <T> Service Type.
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class Client<T> {

    /** Name of the node */
    private final String name;

    /** Node Handler. */
    private final long nodeHandle;

    /** Client Handler. */
    private final long clientHandle;

    /**
     * Constructor.
     *
     * @param nodeHandle
     * @param serviceName
     */
    public Client(final long nodeHandle, long clientHandle, final String serviceName) {
        this.name = serviceName;
        this.nodeHandle = nodeHandle;
        this.clientHandle = clientHandle;
    }

    public void dispose() {
        //TODO
    }

    public <U, V> V sendRequest(U request) {
        //TODO
        throw new NotImplementedException();
//        return null;
    }

    public String getServiceName() {
        return this.name;
    }

    public long getClientHandle() {
        return this.clientHandle;
    }

    public boolean waitForService(int i) {
      //TODO
        throw new NotImplementedException();
//        return false;
    }
}

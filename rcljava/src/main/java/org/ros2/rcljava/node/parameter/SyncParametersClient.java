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
package org.ros2.rcljava.node.parameter;

import java.util.ArrayList;
import java.util.List;

import org.ros2.rcljava.QoSProfile;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.topic.Subscription;

/**
 * Parameter Client.
 *
 * @param <T> Service Type.
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class SyncParametersClient {

    public SyncParametersClient(Node node, QoSProfile profileParameter) {
        // TODO Auto-generated constructor stub
    }

    public boolean hasParameter(final String parameterName) {
        return false;
    }

    public ArrayList<ParameterVariant<?>> getParameters(final List<String> list) {
        // TODO Auto-generated method stub
        return null;
    }

    public ArrayList<rcl_interfaces.msg.ParameterType> getParameterTypes(final List<String> parameterNames) {
        return null;
    }

    public ArrayList<rcl_interfaces.msg.SetParametersResult> setParameters(final List<ParameterVariant<?>> list) {
        // TODO Auto-generated method stub
        return null;
    }

    public rcl_interfaces.msg.ListParametersResult listParameters(final List<String> list, int i) {
        // TODO Auto-generated method stub
        return null;
    }

    public Subscription<?> onParameterEvent(final ParameterConsumer parameterConsumer) { // rcl_interfaces.msg.ParameterEvent
        // TODO Auto-generated method stub
        return null;
    }

}

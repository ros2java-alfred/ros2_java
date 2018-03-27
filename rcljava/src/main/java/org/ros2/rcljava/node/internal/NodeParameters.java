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

import java.util.List;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.parameter.ParameterVariant;
import org.ros2.rcljava.node.topic.SubscriptionCallback;

import rcl_interfaces.msg.ListParametersResult;
import rcl_interfaces.msg.ParameterDescriptor;
import rcl_interfaces.msg.SetParametersResult;

/**
 * Interface of Node Parameters stack.
 */
public interface NodeParameters {

    /**
     *
     * @param parameters
     * @return
     */
    List<SetParametersResult> setParameters(final List<ParameterVariant<?>> parameters);

    /**
     *
     * @param parameters
     * @return
     */
    SetParametersResult setParametersAtomically(final List<ParameterVariant<?>> parameters);

    /**
     *
     * @param name
     * @param value
     */
    <T> void setParameterIfNotSet(final String name, final T value);

    /**
     *
     * @param names
     * @return
     */
    List<ParameterVariant<?>> getParameters(final List<String> names);

    /**
     *
     * @param name
     *            The name of the parameter to get.
     * @return
     */
    ParameterVariant<?> getParameter(final String name);

    /**
     * Assign the value of the parameter if set into the parameter argument. If the
     * parameter was not set, then the "parameter" argument is never assigned a
     * value.
     *
     * @param name
     *            The name of the parameter to get.
     * @param parameter
     *            The output where the value of the parameter should be assigned.
     * @return true if the parameter was set, false otherwise
     */
    boolean getParameter(final String name, ParameterVariant<?> parameter);

    /**
     * Get the parameter value, or the "alternative value" if not set, and assign it
     * to "value". If the parameter was not set, then the "value" argument is
     * assigned the "alternative_value". In all cases, the parameter remains not set
     * after this function is called.
     *
     * @param name
     *            The name of the parameter to get.
     * @param value
     *            The output where the value of the parameter should be assigned.
     * @param alternativeParameter
     *            Value to be stored in output if the parameter was not set.
     * @return true if the parameter was set, false otherwise
     */
    boolean getParameterOr(final String name, ParameterVariant<?> value, ParameterVariant<?> alternativeParameter);

    /**
     *
     * @param names
     * @return
     */
    List<ParameterDescriptor> describeParameters(final List<String> names);

    /**
     *
     * @param names
     * @return
     */
    List<Class<?>> getParameterTypes(final List<String> names);

    /**
     *
     * @param names
     * @param depth
     * @return
     */
    ListParametersResult listParameters(final List<String> names, final int depth);

    /**
     * Register the callback for parameter changes. Repeated invocations of this
     * function will overwrite previous callbacks
     *
     * @param User
     *            defined callback function, It is expected to atomically set
     *            parameters.
     */
    <T extends Message> void registerParamChangeCallback(final SubscriptionCallback<T> callback);

}

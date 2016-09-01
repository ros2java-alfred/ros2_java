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
package org.ros2.rcljava.parameter;

/**
 * Parameter Variant.
 *
 * @param <T> Parameter Type.
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class ParameterVariant<T> {

    private final String name;

    private T value;

    public ParameterVariant(String name, T value) {
        this.name = name;
        this.value = value;
    }

    /**
     * @return the key
     */
    public String getName() {
        return name;
    }

    /**
     * @param value the value to set
     */
    public void setValue(T value) {
        this.value = value;
    }

    /**
     * @return the value
     */
    public T getValue() {
        return value;
    }

    public String getTypeName() {
        // TODO Auto-generated method stub
        return null;
    }

    public String valueToString() {
        return this.value.toString();
    }
}

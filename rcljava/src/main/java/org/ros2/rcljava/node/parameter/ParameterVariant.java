/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016 Mickael Gaillard <mick.gaillard@gmail.com>
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

import rcl_interfaces.msg.Parameter;
import rcl_interfaces.msg.ParameterType;
import rcl_interfaces.msg.ParameterValue;

/**
 * Parameter Variant.
 *
 * @param <T> Parameter Type.
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
        return this.value.getClass().getSimpleName();
    }

    public String valueToString() {
        return this.value.toString();
    }

    public ParameterValue toParameterValue() {
        ParameterValue p = new ParameterValue();
        p.setType(ParameterType.PARAMETER_NOT_SET);

        if (Boolean.class.equals(this.value.getClass())) {
            p.setBoolValue((Boolean) this.value);
            p.setType(ParameterType.PARAMETER_BOOL);
        } else

//        if (List<Byte>).class.equals(this.value.getClass())) {
//            p.setBytesValue((Byte) this.value);
//            p.setType(ParameterType.PARAMETER_BYTES);
//        } else

        if (Double.class.equals(this.value.getClass())) {
            p.setDoubleValue((Double) this.value);
            p.setType(ParameterType.PARAMETER_DOUBLE);
        } else

        if (Long.class.equals(this.value.getClass())) {
            p.setIntegerValue((Long) this.value);
            p.setType(ParameterType.PARAMETER_INTEGER);
        } else

        if (String.class.equals(this.value.getClass())) {
            p.setStringValue((String) this.value);
            p.setType(ParameterType.PARAMETER_STRING);
        }

        return p;
    }

    public Parameter toParameter() {
        Parameter result = new Parameter();
        result.setName(this.name);
        result.setValue(this.toParameterValue());

        return result;
    }

    public static ParameterVariant<?> fromParameter(Parameter parameter) {

        ParameterVariant<?> reuslt = null;

        if (ParameterType.PARAMETER_BOOL == parameter.getValue().getType()) {
            reuslt = new ParameterVariant<Boolean>(
                parameter.getName(),
                parameter.getValue().getBoolValue());
        }

//        if (ParameterType.PARAMETER_BYTES == parameter.getValue().getType()) {
//            reuslt = new ParameterVariant<Byte[]>(
//                parameter.getName(),
//                parameter.getValue().getBytesValue());
//        }

        if (ParameterType.PARAMETER_DOUBLE == parameter.getValue().getType()) {
            reuslt = new ParameterVariant<Double>(
                parameter.getName(),
                parameter.getValue().getDoubleValue());
        }

        if (ParameterType.PARAMETER_INTEGER == parameter.getValue().getType()) {
            reuslt = new ParameterVariant<Long>(
                parameter.getName(),
                parameter.getValue().getIntegerValue());
        }

        if (ParameterType.PARAMETER_STRING == parameter.getValue().getType()) {
            reuslt = new ParameterVariant<String>(
                parameter.getName(),
                parameter.getValue().getStringValue());
        }

        return reuslt;
    }
}

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

package org.ros2.rcljava;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.ros2.rcljava.node.parameter.ParameterVariant;

public final class ArgumentParser {

    private final static String ASSIGN = "=";
    private final static String PARAM_NODE = "-node";
    private final static String PARAM_SPACE = "-prefix";

    private String prefix;
    private String nodeName;
    private final Map<String, ParameterVariant<?>> parameters = new ConcurrentHashMap<String, ParameterVariant<?>>();

    public ArgumentParser(final String namespace, final String defaultName, final String[] args) {
        this.prefix = namespace;
        this.nodeName = defaultName;

        if (args != null && defaultName == null) {
            this.parseNodeAndSpaceName(args);
        }

        if (this.prefix == null) {
            this.prefix = "";
        }

        if (args != null) {
            this.parseParameters(args);
        }
    }

    public String getName() {
        return this.nodeName;
    }

    public String getNameSpace() {
        return this.prefix;
    }

    public Map<String, ParameterVariant<?>> getParameters() {
        return this.parameters;
    }

    private void parseNodeAndSpaceName(final String[] args) {
        for (final String arg : args) {
            if (arg.contains(ASSIGN)) {
                final String[] item = arg.split(ASSIGN);

                if (PARAM_NODE.equals(item[0]) && item[1] != null) {
                    this.nodeName = item[1];
                }

                if (PARAM_SPACE.equals(item[0]) && item[1] != null) {
                    this.prefix = item[1];
                }
            }
        }
    }

    private void parseParameters(final String[] args) {
        for (final String arg : args) {
            if (arg.contains(ASSIGN)) {
                final String[] item = arg.split(ASSIGN);

                // Remove dash
                final String keyRaw = item[0].trim();
                final String key = (keyRaw.startsWith("-")) ? keyRaw.substring(1) : keyRaw;
                final String val = item[1].trim();
                //TODO NativeNode.getLog().debug("Parse argument : " + arg + "\t\t key : " + key + "\t\t value : "+ val );

                if (this.parameters.get(key) == null) {
                    ParameterVariant<?> value = null;

                    if (isLong(val)) {
                        value = new ParameterVariant<Long>(key, Long.parseLong(val));
                    } else if (isDouble(val)) {
                        value = new ParameterVariant<Double>(key, Double.parseDouble(val));
                    } else if (isBoolean(val)) {
                        value = new ParameterVariant<Boolean>(key, Boolean.parseBoolean(val));
                    } else if (isInteger(val)) {
                        value = new ParameterVariant<Integer>(key, Integer.parseInt(val));
                    } else
                    {
                        value = new ParameterVariant<String>(key, val);
                    }

                    this.parameters.put(key, value);
                }
            }
        }
    }

    private boolean isInteger(final String value) {
        boolean result = false;

        try {
            Integer.parseInt(value);
            result = true;
        } catch (Exception e) { }

        return result;
    }

    private boolean isBoolean(final String value) {
        boolean result = false;

        try {
            Boolean.parseBoolean(value);
            result = true;
        } catch (Exception e) { }

        return result;
    }

    private boolean isDouble(final String value) {
        boolean result = false;

        try {
            Double.parseDouble(value);
            result = true;
        } catch (Exception e) { }

        return result;
    }

    private boolean isLong(final String value) {
        boolean result = false;

        try {
            Long.parseLong(value);
            result = true;
        } catch (Exception e) { }

        return result;
    }
}

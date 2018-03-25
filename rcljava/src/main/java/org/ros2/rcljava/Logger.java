/* Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Topics;

//import java.io.PrintWriter;
//import java.io.StringWriter;

import org.slf4j.LoggerFactory;

/**
 * <i>Not define in ROS2. Copy of ROS1</i>
 *
 */
public class Logger {

    private final org.slf4j.Logger log;
    private final String name;
    private final Node defaultNode;
    private final Publisher<std_msgs.msg.String> publisher;

    private boolean isDebugEnabled;
    private boolean isErrorEnabled;
    private boolean isFatalEnabled;
    private boolean isInfoEnabled;
    private boolean isTraceEnabled;
    private boolean isWarnEnabled;

    /**
     * Constructor of a dummy logger.
     *
     * This is used when logging is disabled: see `RCLCPP_LOGGING_ENABLED`.
     * This cannot be called directly, see `rclcpp::get_logger` instead.
     *
     * @param defaultNode
     */
    public Logger(final Node defaultNode) {
        this(defaultNode, null);
    }

    /**
     * Constructor of a named logger.
     *
     * This cannot be called directly, see `rclcpp::get_logger` instead.
     *
     * @param defaultNode
     * @param name
     */
    public Logger(final Node defaultNode, String name) {
        this.name = name;
        this.defaultNode = defaultNode;
        this.publisher = this.defaultNode.createPublisher(std_msgs.msg.String.class, Topics.ROSOUT);
        this.log = LoggerFactory.getLogger(
                GraphName.getFullName(
                        this.defaultNode.getNameSpace(),
                        this.defaultNode.getName()));
    }

    /**
     * Get the name of this logger.
     *
     * @return the full name of the logger including any prefixes, or
     *  `null` if this logger is invalid (e.g. because logging is
     *   disabled).
     */
    public String getName() {
        return this.name;
    }

//    public Publisher<rosgraph_msgs.msg.Log> getPublisher() {
//        return this.publisher;
//    }

//    private void publish(byte level, Object message, Throwable throwable) {
//        StringWriter stringWriter = new StringWriter();
//        PrintWriter printWriter = new PrintWriter(stringWriter);
//        throwable.printStackTrace(printWriter);
//        this.publish(level, message.toString() + '\n' + stringWriter.toString());
//    }

    private void publish(byte level, Object message) {
//        rosgraph_msgs.msg.Log logMessage = new rosgraph_msgs.msg.Log();
//        logMessage.getHeader().setStamp(this.defaultNode.getCurrentTime());
//        logMessage.setLevel(level);
//        logMessage.setName(this.defaultNode.getName());
//        logMessage.setMsg(message.toString());
        std_msgs.msg.String logMessage = new std_msgs.msg.String();
        logMessage.setData(message.toString());
        this.publisher.publish(logMessage);
    }

    public boolean isDebugEnabled() {
        return this.isDebugEnabled;
    }

    public boolean isErrorEnabled() {
        return this.isErrorEnabled;
    }

    public boolean isFatalEnabled() {
        return this.isFatalEnabled;
    }

    public boolean isInfoEnabled() {
        return this.isInfoEnabled;
    }

    public boolean isTraceEnabled() {
        return this.isTraceEnabled;
    }

    public boolean isWarnEnabled() {
        return this.isWarnEnabled;
    }

    public void trace(Object message) {
        this.log.trace(message.toString());
        if (this.isTraceEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.DEBUG, message);
        }
    }

    public void trace(Object message, Throwable t) {
        this.log.trace(message.toString());
        if (this.isTraceEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.DEBUG, message, t);
        }
    }

    public void debug(Object message) {
        this.log.debug(message.toString());
        if (this.isDebugEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.DEBUG, message);
        }
    }

    public void debug(Object message, Throwable t) {
        this.log.debug(message.toString());
        if (this.isDebugEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.DEBUG, message, t);
        }
    }

    public void info(Object message) {
        this.log.info(message.toString());
        if (this.isInfoEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.INFO, message);
        }
    }

    public void info(Object message, Throwable t) {
        this.log.info(message.toString());
        if (this.isInfoEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.INFO, message, t);
        }
    }

    public void warn(Object message) {
        this.log.warn(message.toString());
        if (this.isWarnEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.WARN, message);
        }
    }

    public void warn(Object message, Throwable t) {
        this.log.warn(message.toString());
        if (this.isWarnEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.msg.Log.WARN, message, t);
        }
    }

    public void error(Object message) {
        this.log.error(message.toString());
        if (this.isErrorEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.ERROR, message);
        }
    }

    public void error(Object message, Throwable t) {
        this.log.error(message.toString());
        if (this.isErrorEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.ERROR, message, t);
        }
    }

    public void fatal(Object message) {
        this.log.error(message.toString());
        if (this.isFatalEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.FATAL, message);
        }
    }

    public void fatal(Object message, Throwable t) {
        this.log.error(message.toString());
        if (this.isFatalEnabled() && this.publisher != null) {
            this.publish((byte)0, message);
//            this.publish(rosgraph_msgs.Log.FATAL, message, t);
        }
    }

}

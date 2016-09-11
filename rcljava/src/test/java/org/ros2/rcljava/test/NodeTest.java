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
package org.ros2.rcljava.test;

import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;
import org.ros2.rcljava.Node;
import org.ros2.rcljava.RCLJava;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

/**
 *
 * @author Mickael Gaillard <mick.gaillard@gmail.com>
 */
public class NodeTest {
    private static Logger logger = Logger.getLogger(RCLJava.LOG_NAME);

    @BeforeClass
    public static void setUp() {
        logger.setLevel(Level.ALL);
        ConsoleHandler handler = new ConsoleHandler();
        handler.setFormatter(new SimpleFormatter());
        logger.addHandler(handler);
        handler.setLevel(Level.ALL);
    }

    @Test
    public void testCreateNode() {
        boolean test = true;
        Node node = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testNode");
        } catch (Exception e) {
            test = false;
        }

        RCLJava.shutdown();
        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", node, node);
    }

//    @Ignore
//    @Test
//    public void testPublisher() {
//        boolean test = true;
//        Node node = null;
//        Publisher<std_msgs.msg.String> pub = null;
//
//        RCLJava.rclJavaInit();
//        std_msgs.msg.String msg = new std_msgs.msg.String();
//        try {
//            node = RCLJava.createNode("testPublisher");
//            pub = node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class, "testChannel", QoSProfile.PROFILE_DEFAULT);
//
//        } catch (Exception e) {
//            test = false;
//        }
//
//        RCLJava.shutdown();
//        Assert.assertTrue("Expected Runtime error.", test);
//        Assert.assertNotNull("Bad result", pub);
//    }
}

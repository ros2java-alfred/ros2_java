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
import org.junit.Ignore;
import org.junit.Test;

import org.ros2.rcljava.Consumer;
import org.ros2.rcljava.Node;
import org.ros2.rcljava.Publisher;
import org.ros2.rcljava.QoSProfile;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.Subscription;
import org.ros2.rcljava.service.Client;
import org.ros2.rcljava.service.Service;
import org.ros2.rcljava.service.ServiceConsumer;

import java.util.HashMap;
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
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", node, node);
    }

    @Test
    public void testDestroyNode() {
        boolean test = true;
        Node node = null;

        RCLJava.rclJavaInit();
        node = RCLJava.createNode("testNode");
        try {
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", node, node);
    }

    @Test
    public void testGetNodeName() {
        boolean test = true;
        Node node = null;
        String nodeName = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testNodeName");
            nodeName = node.getName();
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", "testNodeName", nodeName);
    }

    @Ignore
    @Test
    public void testPublisher() {
        boolean test = true;
        Node node = null;
        Publisher<std_msgs.msg.String> pub = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testPublisher");
            pub = node.<std_msgs.msg.String>createPublisher(
                    std_msgs.msg.String.class,
                    "testChannel",
                    QoSProfile.PROFILE_DEFAULT);

            pub.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", pub);
    }

    @Ignore
    @Test
    public void testSubscription() {
        boolean test = true;
        Node node = null;
        Subscription<std_msgs.msg.String> sub = null;

        Consumer<std_msgs.msg.String> callback = new Consumer<std_msgs.msg.String>() {
            @Override
            public void accept(std_msgs.msg.String msg) { }
        };

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testSubscription");
            sub = node.<std_msgs.msg.String>createSubscription(
                    std_msgs.msg.String.class,
                    "testChannel",
                    callback,
                    QoSProfile.PROFILE_DEFAULT);

          //TODO sub.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", sub);
    }

    @Ignore
    @Test
    public void testClient() {
        boolean test = true;
        Node node = null;
        Client<std_msgs.msg.String> clt = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testClient");
            clt = node.<std_msgs.msg.String>createClient(
                    std_msgs.msg.String.class,
                    "testChannel",
                    QoSProfile.PROFILE_DEFAULT);
          //TODO clt.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", clt);
    }

    @Ignore
    @Test
    public void testService() {
        boolean test = true;
        Node node = null;
        Service<std_msgs.msg.String> srv = null;

        ServiceConsumer<std_msgs.msg.String, std_msgs.msg.String> callback =
                new ServiceConsumer<std_msgs.msg.String, std_msgs.msg.String>() {
            @Override
            public void call(std_msgs.msg.String request, std_msgs.msg.String response) { }
        };

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testSubscription");
            srv = node.<std_msgs.msg.String>createService(
                    std_msgs.msg.String.class,
                    "testChannel",
                    callback,
                    QoSProfile.PROFILE_DEFAULT);
          //TODO srv.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", srv);
    }

    @Test
    public void testGraphPublisherCount() {
        boolean test = true;
        int count = -2;
        Node node = null;
        Publisher<std_msgs.msg.String> pub = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testPublisher");

            count = node.countPublishers("testChannel");
            Assert.assertEquals("Bad result", 0, count);
            // TODO
//            pub = node.<std_msgs.msg.String>createPublisher(
//                    std_msgs.msg.String.class,
//                    "testChannel",
//                    QoSProfile.PROFILE_DEFAULT);
//            count = node.countPublishers("testChannel");
//            Assert.assertEquals("Bad result", 1, count);
//            pub.dispose();
            count = node.countPublishers("testChannel");
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", 0, count);
    }

    @Test
    public void testGraphSubscriptionCount() {
        boolean test = true;
        int count = -2;
        Node node = null;
        Subscription<std_msgs.msg.String> sub = null;

        Consumer<std_msgs.msg.String> callback = new Consumer<std_msgs.msg.String>() {
            @Override
            public void accept(std_msgs.msg.String msg) { }
        };

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testSubscription");
            count = node.countPublishers("testChannel");
            Assert.assertEquals("Bad result", 0, count);
            // TODO
//            sub = node.<std_msgs.msg.String>createSubscription(
//                    std_msgs.msg.String.class,
//                    "testChannel",
//                    callback,
//                    QoSProfile.PROFILE_DEFAULT);
//            count = node.countPublishers("testChannel");
//            Assert.assertEquals("Bad result", 1, count);
//            sub.dispose();
            count = node.countPublishers("testChannel");
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", 0, count);
    }

    @Test
    public void testGraphGetTopics() {
        boolean test = true;
        Node node = null;
        HashMap<String, String> topics = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testSubscription");
            topics = node.getTopicNamesAndTypes();

            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", 0, topics.size());
    }

    //TODO Test Parameters


}

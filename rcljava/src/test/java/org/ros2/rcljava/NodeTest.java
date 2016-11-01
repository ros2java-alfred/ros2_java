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
package org.ros2.rcljava;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.service.RMWRequestId;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.TriConsumer;
import org.ros2.rcljava.node.topic.Consumer;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.qos.QoSProfile;

import java.lang.ref.WeakReference;
import java.util.HashMap;

public class NodeTest {

    public class TestConsumer implements Consumer<std_msgs.msg.String> {
        private final RCLFuture<std_msgs.msg.String> future;

        TestConsumer(final RCLFuture<std_msgs.msg.String> future) {
            this.future = future;
        }

        public final void accept(final std_msgs.msg.String msg) {
            if (!this.future.isDone()) {
                this.future.set(msg);
            }
        }
    }

    @Test
    public final void testCreate() {
        boolean test = true;
        Node node = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("test_node");
            node.dispose();
        } catch (Exception e) {
            test = false;
        }
        RCLJava.shutdown();

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotEquals(0, node.getNodeHandle());
    }

    @Test
    public void testDestroyNode() {
        boolean test = true;
        Node node = null;

        RCLJava.rclJavaInit();
        node = RCLJava.createNode("test_node");
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

    @Test
    public final void testPubSub() throws Exception {
        RCLJava.rclJavaInit();
        Node node = RCLJava.createNode("test_node");
        assertNotEquals(0, node.getNodeHandle());

        Publisher<std_msgs.msg.String> publisher = node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class,
                "test_topic");

        RCLFuture<std_msgs.msg.String> future = new RCLFuture<std_msgs.msg.String>(new WeakReference<Node>(node));

//        Subscription<std_msgs.msg.String> subscription =
        node.<std_msgs.msg.String>createSubscription(
                std_msgs.msg.String.class, "test_topic", new TestConsumer(future));

        std_msgs.msg.String msg = new std_msgs.msg.String();
        msg.setData("Hello");

        while (RCLJava.ok() && !future.isDone()) {
            publisher.publish(msg);
            RCLJava.spinOnce(node);
        }

        std_msgs.msg.String value = future.get();
        assertEquals("Hello", value.getData());
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
                    QoSProfile.DEFAULT);

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
                    QoSProfile.DEFAULT);

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
                    QoSProfile.DEFAULT);
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

        TriConsumer<RMWRequestId, std_msgs.msg.String, std_msgs.msg.String> callback =
                new TriConsumer<RMWRequestId, std_msgs.msg.String, std_msgs.msg.String>() {
            @Override
            public void accept(RMWRequestId header, std_msgs.msg.String request, std_msgs.msg.String response) { }
        };

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testSubscription");
            srv = node.<std_msgs.msg.String>createService(
                    std_msgs.msg.String.class,
                    "testChannel",
                    callback,
                    QoSProfile.DEFAULT);
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
//        Publisher<std_msgs.msg.String> pub = null;

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
//        Subscription<std_msgs.msg.String> sub = null;

//        Consumer<std_msgs.msg.String> callback = new Consumer<std_msgs.msg.String>() {
//            @Override
//            public void accept(std_msgs.msg.String msg) { }
//        };

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

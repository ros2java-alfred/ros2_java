/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016-2017 Mickael Gaillard <mick.gaillard@gmail.com>
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

import org.apache.log4j.BasicConfigurator;

import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.service.RMWRequestId;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.node.topic.NativePublisher;
import org.ros2.rcljava.node.topic.NativeSubscription;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.qos.QoSProfile;

import std_msgs.msg.UInt32;

import java.lang.ref.WeakReference;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executor.MultiThreadedExecutor;
import org.ros2.rcljava.executor.SingleThreadedExecutor;
import org.ros2.rcljava.executor.ThreadedExecutor;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.namespace.GraphName;


public class NodeTest {

    public class TestConsumer<T extends Message> implements SubscriptionCallback<T> {
        private final RCLFuture<T> future;

        TestConsumer(final RCLFuture<T> future) {
            this.future = future;
        }

        public final void dispatch(final T msg) {
            if (!this.future.isDone()) {
                this.future.set(msg);
            }
        }
    }

    @BeforeClass
    public static void beforeClass() {
        BasicConfigurator.resetConfiguration();
        BasicConfigurator.configure();
    }

    @Test
    public final void testCreate() {
        boolean test = true;
        Node node = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("_test_node");
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

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
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", node, node);
    }

    @Test
    public void testGetNodeName() {
        boolean test = true;
        Node node = null;
        String nodeName = null;

        try {
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testNodeName");
            nodeName = node.getName();
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

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

        Subscription<std_msgs.msg.String> subscription = node.<std_msgs.msg.String>createSubscription(
                std_msgs.msg.String.class, "test_topic", new TestConsumer<std_msgs.msg.String>(future));

        std_msgs.msg.String msg = new std_msgs.msg.String();
        msg.setData("Hello");

        while (RCLJava.ok() && !future.isDone()) {
            publisher.publish(msg);
            RCLJava.spinOnce(node);
        }

        std_msgs.msg.String value = future.get();
        assertEquals("Hello", value.getData());

        subscription.dispose();
        node.dispose();
        RCLJava.shutdown();
    }

    @Test
    public void testPublisher() {
        boolean test = true;
        Node node = null;
        Publisher<std_msgs.msg.String> pub = null;


        try {
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testPublisher");
//            RCLFuture<std_msgs.msg.String> future = new RCLFuture<std_msgs.msg.String>(new WeakReference<Node>(node));
            pub = node.<std_msgs.msg.String>createPublisher(
                    std_msgs.msg.String.class,
                    "testChannel",
                    QoSProfile.DEFAULT);

//            std_msgs.msg.String msg = new std_msgs.msg.String();
//            msg.setData("Hello");
//
//            while (RCLJava.ok() && !future.isDone()) {
//                pub.publish(msg);
//                RCLJava.spinOnce(node);
//            }

            pub.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", pub);
    }

    @Test
    public void testSubscription() {
        boolean test = true;
        Node node = null;
        Subscription<std_msgs.msg.String> sub = null;

        SubscriptionCallback<std_msgs.msg.String> callback = new SubscriptionCallback<std_msgs.msg.String>() {
            @Override
            public void dispatch(std_msgs.msg.String msg) { }
        };

        try {
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testSubscription");
            sub = node.<std_msgs.msg.String>createSubscription(
                    std_msgs.msg.String.class,
                    "testChannel",
                    callback,
                    QoSProfile.DEFAULT);

            sub.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", sub);
    }

    @Test
    public void testClient() {
        boolean test = true;
        Node node = null;
        Client<rcl_interfaces.srv.GetParameters> clt = null;

        try {
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testClient");
            clt = node.<rcl_interfaces.srv.GetParameters>createClient(
                    rcl_interfaces.srv.GetParameters.class,
                    "testChannel",
                    QoSProfile.DEFAULT);

            clt.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", clt);
    }

    @Test
    public void testService() {
        boolean test = true;
        Node node = null;
        Service<rcl_interfaces.srv.GetParameters> srv = null;

        ServiceCallback<rcl_interfaces.srv.GetParameters_Request, rcl_interfaces.srv.GetParameters_Response> callback =
                new ServiceCallback<rcl_interfaces.srv.GetParameters_Request, rcl_interfaces.srv.GetParameters_Response>() {
            @Override
            public void dispatch(RMWRequestId header, rcl_interfaces.srv.GetParameters_Request request, rcl_interfaces.srv.GetParameters_Response response) { }
        };

        try {
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testSubscription");
            srv = node.<rcl_interfaces.srv.GetParameters>createService(
                    rcl_interfaces.srv.GetParameters.class,
                    "testChannel",
                    callback,
                    QoSProfile.DEFAULT);

            srv.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", srv);
    }

    @Test
    public void testGraphPublisherCount() {
        boolean test = true;
        int count = -2;
        Node node = null;
        Publisher<std_msgs.msg.String> pub = null;
        final String topicPath = "/testChannel";

        try {
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testPublisher");

            count = node.countPublishers(topicPath);
            Assert.assertEquals("Bad result", 0, count);

            pub = node.<std_msgs.msg.String>createPublisher(
                    std_msgs.msg.String.class,
                    topicPath,
                    QoSProfile.DEFAULT);
            count = node.countPublishers(topicPath);
            Assert.assertEquals("Bad result", 1, count);

            pub.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
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

        try {
            RCLJava.rclJavaInit();
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
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", 0, count);
    }

    @Test
    @Ignore
    public void testGraphGetTopics() {
        boolean test = true;
        Node node = null;
        HashMap<String, List<String>> topics = null;
        String fqnNode = null;

        try {
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testSubscription");
            topics = node.getTopicNamesAndTypes();
            fqnNode = GraphName.getFullName(node.getNameSpace(), node.getName());

            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        int i = 0;
        for (Entry<String, List<String>> topic : topics.entrySet()) {
            if (topic.getKey().startsWith(fqnNode)) {
                ++i;
            }
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", 13, i);
    }

    //TODO Test Parameters

    @Test
    public final void testPubUInt32LinkMultipleProcess() throws Exception {
        boolean test = true;

        try {
            RCLJava.rclJavaInit();
            ThreadedExecutor executor = new MultiThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode("publisher_node");
            final Node subscriptionNodeOne  = RCLJava.createNode("subscription_node_one");
            final Node subscriptionNodeTwo  = RCLJava.createNode("subscription_node_two");

            NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, "test_topic_multiple");

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionOne = (NativeSubscription<UInt32>) subscriptionNodeOne.<UInt32>createSubscription(
                    UInt32.class, "test_topic_multiple", new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionTwo = (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                    UInt32.class, "test_topic_multiple", new TestConsumer<UInt32>(futureTwo));

            UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

//            executor.spin();

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
                executor.spinOnce(0);
            }

            UInt32 valueOne = futureOne.get();
            assertEquals(54321, valueOne.getData());

            UInt32 valueTwo = futureTwo.get();
            assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            publisher.dispose();
            subscriptionOne.dispose();
            subscriptionTwo.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
    }

    @Test
    public final void testPubUInt32SeparateMultipleProcess() throws Exception {
        boolean test = true;

        try {
            RCLJava.rclJavaInit();
            ThreadedExecutor executor = new MultiThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode("publisher_node");
            final Node subscriptionNodeOne  = RCLJava.createNode("subscription_node_one");
            final Node subscriptionNodeTwo  = RCLJava.createNode("subscription_node_two");

            NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, "test_topic_multiple");

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionOne = (NativeSubscription<UInt32>) subscriptionNodeOne.<UInt32>createSubscription(
                    UInt32.class, "test_topic_multiple", new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionTwo = (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                    UInt32.class, "test_topic_multiple", new TestConsumer<UInt32>(futureTwo));

            UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            executor.spin();

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
            }

            UInt32 valueOne = futureOne.get();
            assertEquals(54321, valueOne.getData());

            UInt32 valueTwo = futureTwo.get();
            assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            publisher.dispose();
            subscriptionOne.dispose();
            subscriptionTwo.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
    }

    @Test
    public final void testPubUInt32SeparateSingleProcess() throws Exception {
        boolean test = true;

        try {
            RCLJava.rclJavaInit();
            ThreadedExecutor executor = new SingleThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode("publisher_node");
            final Node subscriptionNodeOne  = RCLJava.createNode("subscription_node_one");
            final Node subscriptionNodeTwo  = RCLJava.createNode("subscription_node_two");

            NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, "test_topic_single");

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionOne = (NativeSubscription<UInt32>) subscriptionNodeOne.<UInt32>createSubscription(
                    UInt32.class, "test_topic_single", new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionTwo = (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                    UInt32.class, "test_topic_single", new TestConsumer<UInt32>(futureTwo));

            UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            executor.spin();

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
            }

            UInt32 valueOne = futureOne.get();
            assertEquals(54321, valueOne.getData());

            UInt32 valueTwo = futureTwo.get();
            assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            publisher.dispose();
            subscriptionOne.dispose();
            subscriptionTwo.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
    }

    @Test
    public final void testPubUInt32LinkSingleProcess() throws Exception {
        boolean test = true;

        try {
            RCLJava.rclJavaInit();
            ThreadedExecutor executor = new SingleThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode("publisher_node");
            final Node subscriptionNodeOne  = RCLJava.createNode("subscription_node_one");
            final Node subscriptionNodeTwo  = RCLJava.createNode("subscription_node_two");

            NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, "test_topic_single");

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionOne = (NativeSubscription<UInt32>) subscriptionNodeOne.<UInt32>createSubscription(
                    UInt32.class, "test_topic_single", new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            NativeSubscription<UInt32> subscriptionTwo = (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                    UInt32.class, "test_topic_single", new TestConsumer<UInt32>(futureTwo));

            UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
                executor.spinOnce(0);
            }

            UInt32 valueOne = futureOne.get();
            assertEquals(54321, valueOne.getData());

            UInt32 valueTwo = futureTwo.get();
            assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            publisher.dispose();
            subscriptionOne.dispose();
            subscriptionTwo.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
    }
}

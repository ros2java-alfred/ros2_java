/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.node;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.lang.ref.WeakReference;
import java.util.Map;
import java.util.List;
import java.util.Map.Entry;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import org.ros2.rcljava.AbstractRosTest;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.namespace.GraphName;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;
import org.ros2.rcljava.qos.QoSProfile;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class NodeTest extends AbstractRosTest {

    private static final Logger logger = LoggerFactory.getLogger(NodeTest.class);
    private static final String ERROR_RT = "Expected Runtime error.";
    private static final String ERROR_BAD = "Bad result";

    @Before
    public void setUp() {
        // Disable default setUp.
    }

    @After
    public void tearDown() {
        // Disable default tearDown.
    }

    @Test
    public final void testCreate() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        NativeNode node = null;

        this.initRCLjava();
        try {
            node = (NativeNode) RCLJava.createNode("_test_node");
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RT, test);
        Assert.assertNotEquals(0, node.getNodeHandle());
    }

    @Test
    public void testDestroyNode() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;

        this.initRCLjava();
        node = RCLJava.createNode("test_node");
        try {
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RT, test);
        Assert.assertEquals(ERROR_BAD, node, node);
    }

    @Test
    public void testGetNodeName() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;
        String nodeName = null;

        try {
            this.initRCLjava();
            node = RCLJava.createNode("testNodeName");
            nodeName = node.getName();
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RT, test);
        Assert.assertEquals(ERROR_BAD, "testNodeName", nodeName);
    }

    @Test
    public final void testPubSub() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        this.initRCLjava();
        NativeNode node = (NativeNode) RCLJava.createNode("test_node");
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
        node.close();
        this.releaseRCLjava();
    }

    @Test
    public void testPublisher() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;

        this.initRCLjava();

        Publisher<std_msgs.msg.String> pub = null;

        try {
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
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RT, test);
        Assert.assertNotNull(ERROR_BAD, pub);
    }

    @Test
    public void testSubscription() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;

        this.initRCLjava();

        Subscription<std_msgs.msg.String> sub = null;

        SubscriptionCallback<std_msgs.msg.String> callback = new SubscriptionCallback<std_msgs.msg.String>() {
            @Override
            public void dispatch(std_msgs.msg.String msg) { }
        };

        try {
            node = RCLJava.createNode("testSubscription");
            sub = node.<std_msgs.msg.String>createSubscription(
                    std_msgs.msg.String.class,
                    "testChannel",
                    callback,
                    QoSProfile.DEFAULT);

            sub.dispose();
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RT, test);
        Assert.assertNotNull(ERROR_BAD, sub);
    }

    @Test
    public void testGraphPublisherCount() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        int count = -2;
        Node node = null;
        Publisher<std_msgs.msg.String> pub = null;
        final String topicPath = "/testChannel";

        try {
            this.initRCLjava();
            node = RCLJava.createNode("testPublisher");

            count = node.countPublishers(topicPath);
            Assert.assertEquals(ERROR_BAD, 0, count);

            pub = node.<std_msgs.msg.String>createPublisher(
                    std_msgs.msg.String.class,
                    topicPath,
                    QoSProfile.DEFAULT);
            count = node.countPublishers(topicPath);
            Assert.assertEquals(ERROR_BAD, 1, count);

            pub.dispose();
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RT, test);
    }

    @Test
    public void testGraphSubscriptionCount() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        int count = -2;
        Node node = null;
//        Subscription<std_msgs.msg.String> sub = null;

//        Consumer<std_msgs.msg.String> callback = new Consumer<std_msgs.msg.String>() {
//            @Override
//            public void accept(std_msgs.msg.String msg) { }
//        };

        try {
            this.initRCLjava();
            node = RCLJava.createNode("testSubscription");
            count = node.countPublishers("testChannel");
            Assert.assertEquals(ERROR_BAD, 0, count);
            // TODO
//            sub = node.<std_msgs.msg.String>createSubscription(
//                    std_msgs.msg.String.class,
//                    "testChannel",
//                    callback,
//                    QoSProfile.PROFILE_DEFAULT);
//            count = node.countPublishers("testChannel");
//            Assert.assertEquals(ERROR_BAD, 1, count);
//            sub.dispose();
            count = node.countPublishers("testChannel");
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RT, test);
        Assert.assertEquals(ERROR_BAD, 0, count);
    }

    @Test
    public void testGraphGetTopics() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;
        Map<String, List<String>> topics = null;
        String fqnNode = null;

        try {
            this.initRCLjava();
            node = RCLJava.createNode("testSubscription");
            topics = node.getTopicNamesAndTypes();
            fqnNode = GraphName.getFullName(node.getNameSpace(), node.getName());

            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        int i = 0;
        for (Entry<String, List<String>> topic : topics.entrySet()) {
            if (topic.getKey().startsWith(fqnNode)) {
                ++i;
            }
        }

        Assert.assertTrue(ERROR_RT, test);
        Assert.assertEquals(ERROR_BAD, 1, i);
    }

    //TODO Test Parameters
}

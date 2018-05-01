/* Copyright 2017-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

package org.ros2.rcljava.executor;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;
import org.ros2.rcljava.AbstractRosTest;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.topic.NativePublisher;
import org.ros2.rcljava.node.topic.NativeSubscription;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import std_msgs.msg.UInt32;

public class ExecutorTest extends AbstractRosTest {

    private static final Logger logger = LoggerFactory.getLogger(ExecutorTest.class);

    private static final String TOPIC_PUB = "publisher_node";
    private static final String TOPIC_SUB1 = "subscription_node_one";
    private static final String TOPIC_SUB2 = "subscription_node_two";
    private static final String TEST_TOPIC_MULTI = "test_topic_multiple";
    private static final String TEST_TOPIC_SINGLE = "test_topic_single";

    private static final String ERROR_MSG = "Expected Runtime error.";

    @Rule
    public Timeout globalTimeout = Timeout.seconds(30); // 30 seconds max per method tested


    @Test
    public final void testLinkMultipleProcess() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            final ThreadedExecutor executor = new MultiThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode(TOPIC_PUB);
            final Node subscriptionNodeOne  = RCLJava.createNode(TOPIC_SUB1);
            final Node subscriptionNodeTwo  = RCLJava.createNode(TOPIC_SUB2);

            final NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, TEST_TOPIC_MULTI);

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionOne =
                    (NativeSubscription<UInt32>)subscriptionNodeOne.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_MULTI, new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionTwo =
                    (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_MULTI, new TestConsumer<UInt32>(futureTwo));

            final UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
                executor.spinOnce(0);
            }

            final UInt32 valueOne = futureOne.get();
            Assert.assertEquals(54321, valueOne.getData());

            final UInt32 valueTwo = futureTwo.get();
            Assert.assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            subscriptionTwo.dispose();
            subscriptionOne.dispose();
            publisher.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_MSG, test);
    }

    @Test
    public final void testSeparateMultipleProcess() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            final ThreadedExecutor executor = new MultiThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode(TOPIC_PUB);
            final Node subscriptionNodeOne  = RCLJava.createNode(TOPIC_SUB1);
            final Node subscriptionNodeTwo  = RCLJava.createNode(TOPIC_SUB2);

            final NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, TEST_TOPIC_MULTI);

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionOne =
                    (NativeSubscription<UInt32>) subscriptionNodeOne.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_MULTI, new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionTwo =
                    (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_MULTI, new TestConsumer<UInt32>(futureTwo));

            final UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            executor.spin();

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
            }

            final UInt32 valueOne = futureOne.get();
            Assert.assertEquals(54321, valueOne.getData());

            final UInt32 valueTwo = futureTwo.get();
            Assert.assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            subscriptionTwo.dispose();
            subscriptionOne.dispose();
            publisher.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_MSG, test);
    }

    @Test
    public final void testSeparateSingleProcess() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            final ThreadedExecutor executor = new SingleThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode(TOPIC_PUB);
            final Node subscriptionNodeOne  = RCLJava.createNode(TOPIC_SUB1);
            final Node subscriptionNodeTwo  = RCLJava.createNode(TOPIC_SUB2);

            final NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, TEST_TOPIC_SINGLE);

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionOne =
                    (NativeSubscription<UInt32>) subscriptionNodeOne.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_SINGLE, new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionTwo =
                    (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_SINGLE, new TestConsumer<UInt32>(futureTwo));

            final UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            executor.spin();

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
            }

            final UInt32 valueOne = futureOne.get();
            Assert.assertEquals(54321, valueOne.getData());

            final UInt32 valueTwo = futureTwo.get();
            Assert.assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            subscriptionTwo.dispose();
            subscriptionOne.dispose();
            publisher.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_MSG, test);
    }

    @Test
    public final void testLinkSingleProcess() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            final ThreadedExecutor executor = new SingleThreadedExecutor();

            final Node publisherNode        = RCLJava.createNode(TOPIC_PUB);
            final Node subscriptionNodeOne  = RCLJava.createNode(TOPIC_SUB1);
            final Node subscriptionNodeTwo  = RCLJava.createNode(TOPIC_SUB2);

            final NativePublisher<UInt32> publisher = (NativePublisher<UInt32>) publisherNode.<UInt32>createPublisher(
                    UInt32.class, TEST_TOPIC_SINGLE);

            final RCLFuture<UInt32> futureOne = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionOne =
                    (NativeSubscription<UInt32>) subscriptionNodeOne.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_SINGLE, new TestConsumer<UInt32>(futureOne));

            final RCLFuture<UInt32> futureTwo = new RCLFuture<UInt32>(executor);

            final NativeSubscription<UInt32> subscriptionTwo =
                    (NativeSubscription<UInt32>) subscriptionNodeTwo.<UInt32>createSubscription(
                            UInt32.class, TEST_TOPIC_SINGLE, new TestConsumer<UInt32>(futureTwo));

            final UInt32 msg = new UInt32();
            msg.setData(54321);

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            while (RCLJava.ok() && !(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
                executor.spinOnce(0);
            }

            final UInt32 valueOne = futureOne.get();
            Assert.assertEquals(54321, valueOne.getData());

            final UInt32 valueTwo = futureTwo.get();
            Assert.assertEquals(54321, valueTwo.getData());

            executor.removeNode(subscriptionNodeTwo);
            executor.removeNode(subscriptionNodeOne);
            executor.removeNode(publisherNode);
            executor.cancel();

            subscriptionTwo.dispose();
            subscriptionOne.dispose();
            publisher.dispose();

            publisherNode.dispose();
            subscriptionNodeOne.dispose();
            subscriptionNodeTwo.dispose();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_MSG, test);
    }
}

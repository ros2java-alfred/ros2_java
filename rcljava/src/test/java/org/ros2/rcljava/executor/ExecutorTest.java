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

import java.util.concurrent.ExecutionException;

import org.junit.Assert;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.Timeout;
import org.ros2.rcljava.AbstractRosTest;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.RCLJavaTest;
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
    private static final String TEST_TOPIC_MULTI = RCLJavaTest.TEST_TOPIC + "_multiple";
    private static final String TEST_TOPIC_SINGLE = RCLJavaTest.TEST_TOPIC + "_single";

    private static final String ERROR_MSG = "Expected Runtime error.";

    @Rule
    public Timeout globalTimeout = Timeout.seconds(200); // 30 seconds max per method tested

    private Node publisherNode;
    private NativePublisher<UInt32> publisher;
    private UInt32 msg;

    private Node subscriptionNodeOne;
    private NativeSubscription<UInt32> subscriptionOne;
    private RCLFuture<UInt32> futureOne;

    private Node subscriptionNodeTwo;
    private NativeSubscription<UInt32> subscriptionTwo;
    private RCLFuture<UInt32> futureTwo;

    private void processInit(final ThreadedExecutor executor, final String topic)
            throws InterruptedException, ExecutionException {

        this.publisherNode        = RCLJava.createNode(TOPIC_PUB);
        this.subscriptionNodeOne  = RCLJava.createNode(TOPIC_SUB1);
        this.subscriptionNodeTwo  = RCLJava.createNode(TOPIC_SUB2);

        this.publisher = (NativePublisher<UInt32>)this.publisherNode.<UInt32>createPublisher(UInt32.class, topic);

        this.futureOne = new RCLFuture<UInt32>(executor);
        this.subscriptionOne = (NativeSubscription<UInt32>)this.subscriptionNodeOne.<UInt32>createSubscription(
                UInt32.class,
                topic,
                new TestConsumer<UInt32>(this.futureOne));

        this.futureTwo = new RCLFuture<UInt32>(executor);
        this.subscriptionTwo = (NativeSubscription<UInt32>)this.subscriptionNodeTwo.<UInt32>createSubscription(
                UInt32.class,
                topic,
                new TestConsumer<UInt32>(futureTwo));

        this.msg = new UInt32();
        this.msg.setData(54321);

        executor.addNode(this.publisherNode);
        executor.addNode(this.subscriptionNodeOne);
        executor.addNode(this.subscriptionNodeTwo);
    }

    private void processResult(final ThreadedExecutor executor)
            throws InterruptedException, ExecutionException {

        final UInt32 valueOne = this.futureOne.get();
        Assert.assertEquals(54321, valueOne.getData());

        final UInt32 valueTwo = this.futureTwo.get();
        Assert.assertEquals(54321, valueTwo.getData());

        executor.removeNode(this.subscriptionNodeTwo);
        executor.removeNode(this.subscriptionNodeOne);
        executor.removeNode(this.publisherNode);
        executor.cancel();

        this.subscriptionTwo.dispose();
        this.subscriptionOne.dispose();
        this.publisher.dispose();

        this.publisherNode.dispose();
        this.subscriptionNodeOne.dispose();
        this.subscriptionNodeTwo.dispose();
    }

    @Test
    public final void testLinkMultipleProcess() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());
        boolean test = true;

        try {
            final ThreadedExecutor executor = new MultiThreadedExecutor();
            this.processInit(executor, TEST_TOPIC_MULTI);

            while (!(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(this.msg);
                executor.spinOnce(-1);
            }

            this.processResult(executor);
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
            this.processInit(executor, TEST_TOPIC_MULTI);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    executor.spin();
                }
            }).start();

            while (!(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
            }

            this.processResult(executor);
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
            this.processInit(executor, TEST_TOPIC_SINGLE);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    executor.spin();
                }
            }).start();

            while (!(this.futureOne.isDone() && this.futureTwo.isDone())) {
                publisher.publish(msg);
            }

            this.processResult(executor);
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
            this.processInit(executor, TEST_TOPIC_SINGLE);

            while (!(futureOne.isDone() && futureTwo.isDone())) {
                publisher.publish(msg);
                executor.spinOnce(-1);
            }

            this.processResult(executor);
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_MSG, test);
    }
}

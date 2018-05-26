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
import org.junit.Test;

import org.ros2.rcljava.AbstractRosTest;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.topic.NativePublisher;
import org.ros2.rcljava.node.topic.NativeSubscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import std_msgs.msg.UInt32;

public class ExecutorFailTest extends AbstractRosTest {

    protected static final Logger logger = LoggerFactory.getLogger(ExecutorFailTest.class);

    private static final String TOPIC_PUB = "publisher_node";
    private static final String TOPIC_SUB1 = "subscription_node_one";
    private static final String TOPIC_SUB2 = "subscription_node_two";
    private static final String TEST_TOPIC_SINGLE = "test_topic_single";

    private static final String ERROR_MSG = "Expected Runtime error.";

    public class TestConsumerFail<T extends Message> implements SubscriptionCallback<T> {
        private final RCLFuture<T> future;

        TestConsumerFail(final RCLFuture<T> future) {
            this.future = future;
        }

        public final void dispatch(final T msg) {
            if (!this.future.isDone()) {
                this.future.set(msg);

                logger.error(">>>>>>>>>>>>> Controled Fail !!! <<<<<<<<<<<<<<<");
                throw new NullPointerException();
            }
        }
    }

    @Test
    public final void testSeparateSingleProcessFail() {
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
                            UInt32.class, TEST_TOPIC_SINGLE, new TestConsumerFail<UInt32>(futureTwo));

            executor.addNode(publisherNode);
            executor.addNode(subscriptionNodeOne);
            executor.addNode(subscriptionNodeTwo);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    executor.spin();
                }
            }).start();

            final UInt32 msg = new UInt32();
            msg.setData(54321);

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

            subscriptionNodeTwo.dispose();
            subscriptionNodeOne.dispose();
            publisherNode.dispose();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_MSG, test);
    }
}

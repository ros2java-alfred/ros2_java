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

package org.ros2.rcljava;

import java.lang.ref.WeakReference;
import java.lang.reflect.Method;
import java.util.concurrent.ExecutionException;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;
import org.ros2.rcljava.node.topic.SubscriptionCallback;

import std_msgs.msg.Header;

public abstract class AbstractMessageTest extends AbstractRosTest {

    private final static String NODE_NAME       = "test_node";
    private final static String VALUE_FRAMEID   = "frameTest";
    private final static int VALUE_TIME_SEC     = 100;
    private final static int VALUE_TIME_NANOSEC = 200;

    protected Node node;

    @Before
    public void setUp() {
        super.setUp();
        this.node = RCLJava.createNode(NODE_NAME);
    }

    @After
    public void tearDown() {
        this.node.dispose();
        super.tearDown();
    }


    @Test
    public final void testCreate() {
        Assert.assertNotEquals(null, this.node);
    }

    @SuppressWarnings("unchecked")
    private <T extends Message> Header headerInit(final T msg) {
        Header header = null;
        final Class<T> msgClass = (Class<T>) msg.getClass();

        try {
            final Method methodToFind = msgClass.getMethod("getHeader", (Class<?>[]) null);
            if (methodToFind != null) {
                header = (Header) methodToFind.invoke(msg);
                headerInit(header);
            }
        } catch (Exception ignore) {

        }

        if (header != null) {
            header.setFrameId(VALUE_FRAMEID);
            header.getStamp().setSec(VALUE_TIME_SEC);
            header.getStamp().setNanosec(VALUE_TIME_NANOSEC);
        }

        return header;
    }

    private void headerTest(final Header msg) {
        Assert.assertEquals(VALUE_FRAMEID, msg.getFrameId());
        Assert.assertEquals(VALUE_TIME_SEC, msg.getStamp().getSec());
        Assert.assertEquals(VALUE_TIME_NANOSEC, msg.getStamp().getNanosec());
    }

    @SuppressWarnings("unchecked")
    protected <T extends Message> T pubSubTest(final T msg) throws InterruptedException, ExecutionException {
        final Class<T> msgClass = (Class<T>) msg.getClass();
        final Publisher<T> publisher = this.node.createPublisher(msgClass, "test_topic");

        final Header header = this.headerInit(msg);

        final RCLFuture<T> future = new RCLFuture<T>(new WeakReference<Node>(this.node));
        final Subscription<T> subscription = this.node.createSubscription(
                msgClass, "test_topic", new TestConsumer<T>(future));

        while (RCLJava.ok() && !future.isDone()) {
            publisher.publish(msg);
            RCLJava.spinOnce(this.node);
        }

        final T value = future.get();

        publisher.dispose();
        subscription.dispose();

        Assert.assertNotNull(value);

        if (header != null) {
            this.headerTest(header);
        }

        return value;
    }

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
}

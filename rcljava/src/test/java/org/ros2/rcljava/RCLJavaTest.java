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

package org.ros2.rcljava;

import java.lang.ref.WeakReference;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import org.ros2.rcljava.exception.NotInitializedException;
import org.ros2.rcljava.executor.DefaultThreadedExecutor;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.topic.Publisher;
import org.ros2.rcljava.node.topic.Subscription;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import std_msgs.msg.Bool;

//@FixMethodOrder(MethodSorters.JVM)
public class RCLJavaTest extends AbstractRosTest {
    private static final Logger logger = LoggerFactory.getLogger(RCLJavaTest.class);

    public static final String TEST_TOPIC = "_test_topic";

    public static final String ERROR_RUNTIME = "Expected Runtime error.";

    @Before
    public void setUp() {
        // Disable default setUp.
    }

    @After
    public void tearDown() {
        // Disable default tearDown.
    }

    @Test
    public final void testInit() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            Assert.assertEquals(false, RCLJava.isInitialized());
            this.initRCLjava();
            Assert.assertEquals(true, RCLJava.isInitialized());
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue("failed to initialize rclJava", test);
    }

    @Test
    public final void testInitShutdown() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            this.initRCLjava();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue("failed to shutdown rclJava", test);
    }

    @Test
    public final void testInitShutdownSequence() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        this.initRCLjava();
        this.releaseRCLjava();

        try {
            RCLJava.rclJavaInit();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue("failed to initialize rclJava after shutdown", test);
    }

    @Test
    public final void testShutdownDouble() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;

        this.initRCLjava();
        this.releaseRCLjava();

        try {
            this.releaseRCLjava();
        } catch (Exception e) {
            test = true;
        }

        Assert.assertTrue("Expected Runtime error when shutting down rclJava twice", test);
    }

    @Test
    public final void testCreateNode() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;

        this.initRCLjava();
        try {
            node = RCLJava.createNode("testNode");
            node.getName();
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue(ERROR_RUNTIME, test);
    }

    @Test
    public final void testOk() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        this.initRCLjava();

        try {
            test = RCLJava.ok();
            Assert.assertEquals(true, RCLJava.ok());
        } catch (Exception e) {
            test = false;
        }
        Assert.assertTrue(ERROR_RUNTIME, test);

        try {
            this.releaseRCLjava();
            test = RCLJava.ok();
            // not really call...
            Assert.assertEquals(false, test);
            test = false;
        } catch (NotInitializedException e) {
            test = true;
        } catch (Exception e) {
            test = false;
        }
        Assert.assertTrue(ERROR_RUNTIME, test);
    }

    @Test
    public final void testSpin() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            this.initRCLjava();
            final Node node = RCLJava.createNode("_" + RCLJavaTest.class.getSimpleName());
            final Publisher<Bool> publisher = node.createPublisher(Bool.class, RCLJavaTest.TEST_TOPIC);

            final Bool msg = new Bool();
            msg.setData(true);

            final RCLFuture<Bool> future = new RCLFuture<Bool>(new WeakReference<Node>(node));
            final Subscription<Bool> subscription = node.createSubscription(
                    Bool.class, RCLJavaTest.TEST_TOPIC, new TestConsumer<Bool>(future));

            final Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    RCLJava.spin(node);
                }
            });
            thread.start();

            while (RCLJava.ok() && !future.isDone()) {
                publisher.publish(msg);
            }

            final Bool value = future.get();
            Assert.assertNotNull(value);

            publisher.dispose();
            subscription.dispose();

            DefaultThreadedExecutor.getInstance().removeNode(node);

            node.dispose();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_RUNTIME, test);
    }

    @Test
    @Ignore
    public final void testSpinOnce() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            this.initRCLjava();
            final Node node = RCLJava.createNode("_" + RCLJavaTest.class.getSimpleName());
            final Publisher<Bool> publisher = node.createPublisher(Bool.class, RCLJavaTest.TEST_TOPIC);

            final Bool msg = new Bool();
            msg.setData(true);

            final RCLFuture<Bool> future = new RCLFuture<Bool>(new WeakReference<Node>(node));
            final Subscription<Bool> subscription = node.createSubscription(
                    Bool.class, RCLJavaTest.TEST_TOPIC, new TestConsumer<Bool>(future));

            new Thread(new Runnable() {
                @Override
                public void run() {
                    RCLJava.spinOnce(node);
                }
            }).start();

            while (RCLJava.ok() && !future.isDone()) {
                publisher.publish(msg);
            }

            final Bool value = future.get();
            Assert.assertNotNull(value);

            publisher.dispose();
            subscription.dispose();

            node.dispose();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue(ERROR_RUNTIME, test);
    }
}

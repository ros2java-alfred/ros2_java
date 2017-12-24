/* Copyright 2017 Mickael Gaillard <mick.gaillard@gmail.com>
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

import org.apache.log4j.BasicConfigurator;

import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;

import org.ros2.rcljava.internal.message.Message;
import org.ros2.rcljava.node.service.RCLFuture;
import org.ros2.rcljava.node.topic.SubscriptionCallback;

public class AbstractRosTest {

    @BeforeClass
    public static void beforeClass() {
        BasicConfigurator.resetConfiguration();
        BasicConfigurator.configure();
        waitLoading();
    }

    @Before
    public void setUp() {
        this.initRCLjava();
    }

    @After
    public void tearDown() {
        this.releaseRCLjava();
    }

    protected void initRCLjava() {
        RCLJava.rclJavaInit();
        waitLoading();
    }

    protected void releaseRCLjava() {
        RCLJava.shutdown();
        waitLoading();
    }

    protected static void waitLoading() {
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public class TestConsumer<T extends Message> implements SubscriptionCallback<T> {
        private final RCLFuture<T> future;

        public TestConsumer(final RCLFuture<T> future) {
            this.future = future;
        }

        public final void dispatch(final T msg) {
            if (!this.future.isDone()) {
                this.future.set(msg);
            }
        }
    }

}

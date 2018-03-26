/* Copyright 2017 Esteve Fernandez <esteve@apache.org>
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

package org.ros2.rcljava.time;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.lang.ref.WeakReference;
import java.util.concurrent.TimeUnit;

import org.junit.Test;

import org.ros2.rcljava.AbstractRosTest;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.RCLFuture;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TimerTest extends AbstractRosTest {

    private static final Logger logger = LoggerFactory.getLogger(TimerTest.class);

    public static class TimerCallback implements WallTimerCallback {
        private final RCLFuture<Boolean> future;
        private int counter;
        private final int maxCount;

        TimerCallback(final RCLFuture<Boolean> future, final int maxCount) {
            this.future = future;
            this.maxCount = maxCount;
            this.counter = 0;
        }

        public void tick() {
            this.counter++;
            if (this.counter >= this.maxCount) {
                this.future.set(true);
            }
        }

        public int getCounter() {
            return this.counter;
        }
    }

    @Test
    public final void testCreate() {
        logger.debug(new Object() {}.getClass().getEnclosingMethod().getName());

        final int max_iterations = 4;

        final Node node = RCLJava.createNode("test_node");

        final RCLFuture<Boolean> future = new RCLFuture<Boolean>(new WeakReference<Node>(node));
        final TimerCallback timerCallback = new TimerCallback(future, max_iterations);

        final WallTimer timer = node.createWallTimer(250, TimeUnit.MILLISECONDS, timerCallback);
        assertNotEquals(null, timer);

        while (RCLJava.ok() && !future.isDone()) {
            RCLJava.spinOnce(node);
        }

        assertFalse("Time is cancel.", timer.isCanceled());
        timer.cancel();

        assertEquals("Time period not good.", TimeUnit.NANOSECONDS.convert(250, TimeUnit.MILLISECONDS), timer.getTimerPeriodNS());
        assertFalse("Time is ready.", timer.isReady());
        assertTrue("Time is not cancel.", timer.isCanceled());

        assertEquals("Bad count.", 4, timerCallback.getCounter());

        timer.dispose();
        node.dispose();
    }
}

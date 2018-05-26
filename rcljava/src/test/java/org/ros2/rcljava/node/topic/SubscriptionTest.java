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

package org.ros2.rcljava.node.topic;

import org.junit.Assert;
import org.junit.Test;

import org.ros2.rcljava.AbstractRosTest;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.RCLJavaTest;
import org.ros2.rcljava.node.NativeNode;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import std_msgs.msg.String;

public class SubscriptionTest extends AbstractRosTest {

    private static final Logger logger = LoggerFactory.getLogger(SubscriptionTest.class);

    @Test
    public final void testCreate() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final NativeNode node = (NativeNode) RCLJava.createNode("test_node");
        final NativeSubscription<String> subscription =
                (NativeSubscription<String>) node.<String>createSubscription(
                        String.class,
                        RCLJavaTest.TEST_TOPIC,
                        new SubscriptionCallback<String>() {
                            public void dispatch(final String msg) {

                            }
                        });

        Assert.assertEquals(node.getNodeHandle(), subscription.getNode().getNodeHandle());
        Assert.assertNotEquals(0, subscription.getNode().getNodeHandle());
        Assert.assertNotEquals(0, subscription.getSubscriptionHandle());

        subscription.dispose();
        node.dispose();
    }
}

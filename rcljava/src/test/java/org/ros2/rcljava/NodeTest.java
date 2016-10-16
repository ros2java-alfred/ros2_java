/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
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

import org.junit.Test;

import java.lang.ref.WeakReference;

public class NodeTest {

  public class TestConsumer implements Consumer<std_msgs.msg.String> {
    private final RCLFuture future;

    TestConsumer(final RCLFuture<std_msgs.msg.String> future) {
      this.future = future;
    }

    public final void accept(final std_msgs.msg.String msg) {
      if(!this.future.isDone()) {
        this.future.set(msg);
      }
    }
  }

  @Test
  public final void testCreate() {
    RCLJava.rclJavaInit();
    Node node = RCLJava.createNode("test_node");
    assertNotEquals(0, node.getNodeHandle());
  }

  @Test
  public final void testPubSub() throws Exception {
    RCLJava.rclJavaInit();
    Node node = RCLJava.createNode("test_node");
    assertNotEquals(0, node.getNodeHandle());

    Publisher<std_msgs.msg.String> publisher = node
        .<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class,
        "test_topic");

    RCLFuture<std_msgs.msg.String> future = new RCLFuture<std_msgs.msg.String>(new WeakReference<Node>(node));

    Subscription<std_msgs.msg.String> subscription = node
        .<std_msgs.msg.String>createSubscription(std_msgs.msg.String.class,
        "test_topic", new TestConsumer(future));

    std_msgs.msg.String msg = new std_msgs.msg.String();
    msg.setData("Hello");

    while (RCLJava.ok() && !future.isDone()) {
      publisher.publish(msg);
      RCLJava.spinOnce(node);
    }

    std_msgs.msg.String value = future.get();
    assertEquals("Hello", value.getData());
  }
}

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

/**
 * This class serves as a bridge between ROS2's rcl_publisher_t and RCLJava.
 * A Publisher must be created via
 * @{link Node#createPublisher(Class&lt;T&gt;, String)}
 *
 * @param <T> The type of the messages that this publisher will publish.
 */
public class Publisher<T> {
  static {
    try {
      System.loadLibrary("rcljavaPublisher__" + RCLJava.getRMWIdentifier());
    } catch (UnsatisfiedLinkError ule) {
      System.err.println("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  /**
   * An integer that represents a pointer to the underlying ROS2 node
   * structure (rcl_node_t).
   */
  private final long nodeHandle;

  /**
   * An integer that represents a pointer to the underlying ROS2 publisher
   * structure (rcl_publisher_t).
   */
  private final long publisherHandle;

  /**
   * The topic to which this publisher will publish messages.
   */
  private final String topic;

  /**
   * Constructor.
   *
   * @param nodeHandle A pointer to the underlying ROS2 node structure that
   *     created this subscription, as an integer. Must not be zero.
   * @param publisherHandle A pointer to the underlying ROS2 publisher
   *     structure, as an integer. Must not be zero.
   * @param topic The topic to which this publisher will publish messages.
   */
  public Publisher(final long nodeHandle, final long publisherHandle,
      final String topic) {
    this.nodeHandle = nodeHandle;
    this.publisherHandle = publisherHandle;
    this.topic = topic;
  }

  /**
   * Publish a message via the underlying ROS2 mechanisms.
   *
   * @param <T> The type of the messages that this publisher will publish.
   * @param publisherHandle A pointer to the underlying ROS2 publisher
   *     structure, as an integer. Must not be zero.
   * @param message An instance of the &lt;T&gt; parameter.
   */
  private static native <T> void nativePublish(
      long publisherHandle, T message);

  /**
   * Publish a message.
   *
   * @param message An instance of the &lt;T&gt; parameter.
   */
  public final void publish(final T message) {
    nativePublish(this.publisherHandle, message);
  }

  /**
   * Destroy a ROS2 publisher (rcl_publisher_t).
   *
   * @param nodeHandle A pointer to the underlying ROS2 node structure that
   *     created this subscription, as an integer. Must not be zero.
   * @param publisherHandle A pointer to the underlying ROS2 publisher
   *     structure, as an integer. Must not be zero.
   */
  private static native void nativeDispose(
      long nodeHandle, long publisherHandle);

  /**
   * Safely destroy the underlying ROS2 publisher structure.
   */
  public final void dispose() {
    nativeDispose(this.nodeHandle, this.publisherHandle);
  }

  public final long getNodeHandle() {
    return this.nodeHandle;
  }

  public final long getPublisherHandle() {
    return this.publisherHandle;
  }
}

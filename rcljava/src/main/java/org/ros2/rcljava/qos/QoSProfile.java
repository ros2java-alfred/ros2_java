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

package org.ros2.rcljava.qos;

import org.ros2.rcljava.qos.policies.Durability;
import org.ros2.rcljava.qos.policies.History;
import org.ros2.rcljava.qos.policies.QoSPolicy;
import org.ros2.rcljava.qos.policies.Reliability;

public class QoSProfile {

  private final History history;

  private final int depth;

  private final Reliability reliability;

  private final Durability durability;

  public QoSProfile(History history, int depth, Reliability reliability,
      Durability durability) {
    this.history = history;
    this.depth = depth;
    this.reliability = reliability;
    this.durability = durability;
  }

  public final History getHistory() {
    return this.history;
  }

  public final int getDepth() {
    return this.depth;
  }

  public final Reliability getReliability() {
    return this.reliability;
  }

  public final Durability getDurability() {
    return this.durability;
  }

  public static final int DEPTH_SYSTEM_DEFAULT = 0;

  public static final QoSProfile SENSOR_DATA = new QoSProfile(
      History.KEEP_LAST,
      5,
      Reliability.BEST_EFFORT,
      Durability.SYSTEM_DEFAULT
  );

  public static final QoSProfile PARAMETER = new QoSProfile(
      History.KEEP_LAST,
      1000,
      Reliability.RELIABLE,
      Durability.SYSTEM_DEFAULT
   );

  public static final QoSProfile DEFAULT = new QoSProfile(
      History.KEEP_ALL,
      10,
      Reliability.RELIABLE,
      Durability.SYSTEM_DEFAULT
   );

  public static final QoSProfile SERVICES_DEFAULT = new QoSProfile(
      History.KEEP_LAST,
      10,
      Reliability.RELIABLE,
      Durability.TRANSIENT_LOCAL
   );

  public static final QoSProfile PARAMETER_EVENTS = new QoSProfile(
      History.KEEP_ALL,
      1000,
      Reliability.RELIABLE,
      Durability.SYSTEM_DEFAULT
   );

  public static final QoSProfile SYSTEM_DEFAULT = new QoSProfile(
      History.SYSTEM_DEFAULT,
      DEPTH_SYSTEM_DEFAULT,
      Reliability.SYSTEM_DEFAULT,
      Durability.SYSTEM_DEFAULT
   );
}

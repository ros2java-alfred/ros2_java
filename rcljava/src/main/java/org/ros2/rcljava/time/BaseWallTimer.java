/* Copyright 2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

import java.lang.ref.WeakReference;

import org.ros2.rcljava.node.Node;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 */
public abstract class BaseWallTimer implements WallTimer {

    private static final Logger logger = LoggerFactory.getLogger(BaseWallTimer.class);

    private long timerPeriodNS;

    private final WeakReference<Node> nodeReference;

    private final WallTimerCallback callback;

    /**
     *
     * @param nodeReference
     * @param callback
     * @param timerPeriodNS
     */
    public BaseWallTimer(
            final WeakReference<Node> nodeReference,
            final WallTimerCallback callback,
            final long timerPeriodNS) {

        if (nodeReference.get() == null) { throw new RuntimeException("Need to provide active node with handle object"); }
        this.nodeReference = nodeReference;

        this.callback = callback;
        this.timerPeriodNS = timerPeriodNS;
    }

    @Override
    public void close() throws Exception {
        this.dispose();
    }

    @Override
    public void dispose() {
        final Node node = this.nodeReference.get();
        if (node != null) {
            BaseWallTimer.logger.debug("Destroy Timer of node : " + node.getName());
        }
    }

    public final WallTimerCallback getCallback() {
        return this.callback;
    }

    public void setTimerPeriodNS(final long timerPeriodNS) {
        this.timerPeriodNS = timerPeriodNS;
    }

    public long getTimerPeriodNS() {
        return this.timerPeriodNS;
    }

    public Node getNode() {
        return this.nodeReference.get();
    }
}

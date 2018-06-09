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

import org.ros2.rcljava.exception.NotImplementedException;
import org.ros2.rcljava.node.Node;

/**
 * This class is JVM WallTimer of RCLJava.
 * <b>Actually not implemented !!!</b>
 */
public class JavaWallTimer extends BaseWallTimer {

    /**
     *
     * @param nodeReference Node.
     * @param callback Call-back.
     * @param timerPeriodNS Time period in nano-second.
     */
    public JavaWallTimer(
            final WeakReference<Node> nodeReference,
            final WallTimerCallback callback,
            final long timerPeriodNS) {
        super(nodeReference, callback, timerPeriodNS);

        throw new NotImplementedException();
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#isReady()
     */
    @Override
    public boolean isReady() {
        // TODO Auto-generated method stub
        return false;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#isCanceled()
     */
    @Override
    public boolean isCanceled() {
        // TODO Auto-generated method stub
        return false;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#cancel()
     */
    @Override
    public void cancel() {
        // TODO Auto-generated method stub

    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#reset()
     */
    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#timeSinceLastCall()
     */
    @Override
    public long timeSinceLastCall() {
        // TODO Auto-generated method stub
        return 0;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#timeUntilNextCall()
     */
    @Override
    public long timeUntilNextCall() {
        // TODO Auto-generated method stub
        return 0;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#callTimer()
     */
    @Override
    public void callTimer() {
        // TODO Auto-generated method stub

    }

}

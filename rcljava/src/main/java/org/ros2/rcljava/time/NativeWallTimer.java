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

import java.lang.ref.WeakReference;

import org.ros2.rcljava.node.Node;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class NativeWallTimer extends BaseWallTimer {

    private static final Logger logger = LoggerFactory.getLogger(NativeWallTimer.class);

    private final long wallTimerHandle;

    private static native long nativeCreateTimerHandle(long timerPeriod);

    private static native boolean nativeIsReady(long handle);

    private static native boolean nativeIsCanceled(long handle);

    private static native long nativeTimeSinceLastCall(long handle);

    private static native long nativeTimeUntilNextCall(long handle);

    private static native long nativeReset(long handle);

    private static native long nativeCancel(long handle);

    private static native long nativeGetTimerPeriodNS(long handle);

    private static native void nativeSetTimerPeriodNS(long handle, long period);

    private static native long nativeCallTimer(long handle);

    private static native void nativeDispose(long handle);

    /**
     * Constructor.
     *
     * @param nodeReference Node.
     * @param callback Call-back.
     * @param timerPeriodNS Time period in nano-second.
     */
    public NativeWallTimer(
            final WeakReference<Node> nodeReference,
            final WallTimerCallback callback,
            final long timerPeriodNS) {
        super(nodeReference, callback, timerPeriodNS);

        this.wallTimerHandle = NativeWallTimer.nativeCreateTimerHandle(timerPeriodNS);
        if (this.wallTimerHandle == 0) { throw new RuntimeException("Need to provide active node with handle object"); }
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.BaseWallTimer#dispose()
     */
    @Override
    public void dispose() {
        super.dispose();

        final Node node = this.getNode();
        if (node != null) {
            NativeWallTimer.logger.debug("Destroy Timer of node : " + node.getName());
        }
        NativeWallTimer.nativeDispose(this.wallTimerHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#timeSinceLastCall()
     */
    public long timeSinceLastCall() {
        return NativeWallTimer.nativeTimeSinceLastCall(this.wallTimerHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#timeUntilNextCall()
     */
    public long timeUntilNextCall() {
        return NativeWallTimer.nativeTimeUntilNextCall(this.wallTimerHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#reset()
     */
    public void reset() {
        NativeWallTimer.nativeReset(this.wallTimerHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#cancel()
     */
    public void cancel() {
        NativeWallTimer.nativeCancel(this.wallTimerHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#isCanceled()
     */
    public boolean isCanceled() {
        return NativeWallTimer.nativeIsCanceled(this.wallTimerHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#isReady()
     */
    public boolean isReady() {
        return NativeWallTimer.nativeIsReady(this.wallTimerHandle);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.BaseWallTimer#setTimerPeriodNS(long)
     */
    @Override
    public void setTimerPeriodNS(final long timerPeriodNS) {
        NativeWallTimer.nativeSetTimerPeriodNS(this.wallTimerHandle, timerPeriodNS);
        super.setTimerPeriodNS(timerPeriodNS);
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.BaseWallTimer#getTimerPeriodNS()
     */
    @Override
    public long getTimerPeriodNS() {
        final long timerPeriodNS = NativeWallTimer.nativeGetTimerPeriodNS(this.wallTimerHandle);
        this.setTimerPeriodNS(timerPeriodNS);
        return super.getTimerPeriodNS();
    }

    /**
     * @return Handle.
     */
    public long getHandle() {
        return this.wallTimerHandle;
    }

    /* (non-Javadoc)
     * @see org.ros2.rcljava.time.WallTimer#callTimer()
     */
    @Override
    public void callTimer() {
        NativeWallTimer.nativeCallTimer(this.wallTimerHandle);
    }


}

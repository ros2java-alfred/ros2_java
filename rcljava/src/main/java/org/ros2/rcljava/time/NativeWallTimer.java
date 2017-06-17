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

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;

public class NativeWallTimer implements WallTimer, java.lang.AutoCloseable {
    private static final Logger logger = LoggerFactory.getLogger(NativeWallTimer.class);

    static {
        RCLJava.loadLibrary("rcljava_time_NativeWallTimer");
    }

    private long timerPeriodNS;

    private final WeakReference<Node> nodeReference;

    private final long wallTimerHandle;

    private final WallTimerCallback callback;

    private static native boolean nativeIsReady(long handle);

    private static native boolean nativeIsCanceled(long handle);

    private static native long nativeTimeSinceLastCall(long handle);

    private static native long nativeTimeUntilNextCall(long handle);

    private static native long nativeReset(long handle);

    private static native long nativeCancel(long handle);

    private static native long nativeGetTimerPeriodNS(long handle);

    private static native void nativeSetTimerPeriodNS(long handle, long period);

    private static native long nativeCallTimer(long handle);

    public NativeWallTimer(final WeakReference<Node> nodeReference, final long handle, final WallTimerCallback callback,
            final long timerPeriodNS) {
        this.nodeReference = nodeReference;
        this.wallTimerHandle = handle;
        this.callback = callback;
        this.timerPeriodNS = timerPeriodNS;
    }

    public final WallTimerCallback getCallback() {
        return this.callback;
    }

    public long timeSinceLastCall() {
        return NativeWallTimer.nativeTimeSinceLastCall(this.wallTimerHandle);
    }

    public long timeUntilNextCall() {
        return NativeWallTimer.nativeTimeUntilNextCall(this.wallTimerHandle);
    }

    public void reset() {
        NativeWallTimer.nativeReset(this.wallTimerHandle);
    }

    public void cancel() {
        NativeWallTimer.nativeCancel(this.wallTimerHandle);
    }

    public boolean isCanceled() {
        return NativeWallTimer.nativeIsCanceled(this.wallTimerHandle);
    }

    public boolean isReady() {
        return NativeWallTimer.nativeIsReady(this.wallTimerHandle);
    }

    public void setTimerPeriodNS(final long timerPeriodNS) {
        NativeWallTimer.nativeSetTimerPeriodNS(this.wallTimerHandle, timerPeriodNS);
        this.timerPeriodNS = timerPeriodNS;
    }

    public long getTimerPeriodNS() {
        long timerPeriodNS = NativeWallTimer.nativeGetTimerPeriodNS(this.wallTimerHandle);
        this.timerPeriodNS = timerPeriodNS;
        return this.timerPeriodNS;
    }

    public long getHandle() {
        return this.wallTimerHandle;
    }

    private static native void nativeDispose(long handle);

    @Override
    public void dispose() {
        Node node = this.nodeReference.get();
        if (node != null) {
            NativeWallTimer.logger.debug("Destroy Timer of node : " + node.getName());
            NativeWallTimer.nativeDispose(this.wallTimerHandle);
        }
    }

    public void callTimer() {
        NativeWallTimer.nativeCallTimer(this.wallTimerHandle);
    }

    @Override
    public void close() throws Exception {
        this.dispose();
    }
}

package org.ros2.rcljava.time;

public interface WallTimer {

    long getTimerPeriodNS();

    void setTimerPeriodNS(long period);

    boolean isReady();
    boolean isCanceled();

    void cancel();

    void reset();

    long timeSinceLastCall();

    long timeUntilNextCall();

    WallTimerCallback getCallback();

    void callTimer();

    /**
     * Safely destroy the underlying ROS2 publisher structure.
     */
    void dispose();

}

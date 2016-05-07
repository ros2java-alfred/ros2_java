package org.ros2.rcljava;

public class Subscription<T> {

    private long nodeHandle;
    private long subscriptionHandle;
    private Class<T> msgType;
    private String topic;
    private Consumer<T> callback;

    public Subscription(long nodeHandle, long subscriptionHandle, Class<T> msgType, String topic, Consumer<T> callback) {
        this.nodeHandle = nodeHandle;
        this.subscriptionHandle = subscriptionHandle;
        this.msgType = msgType;
        this.topic = topic;
        this.callback = callback;
    }

    public Consumer<T> getCallback() {
        return callback;
    }

    public Class<T> getMsgType() {
        return msgType;
    }

    public long getSubscriptionHandle() {
        return subscriptionHandle;
    }
}

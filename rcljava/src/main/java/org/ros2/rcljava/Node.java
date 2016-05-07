package org.ros2.rcljava;

import java.lang.ref.WeakReference;

import java.util.ArrayList;
import java.util.List;

public class Node {
   static {
        try {
            System.loadLibrary("rcljavaNode__" + RCLJava.getRMWIdentifier());
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.exit(1);
        }
    }

    private long nodeHandle;
    private List<Subscription> subscriptions = new ArrayList<Subscription>();

    public Node(long nodeHandle) {
        this.nodeHandle = nodeHandle;
    }

    private static native <T> long nativeCreatePublisherHandle(
        long nodeHandle, Class<T> cls, String topic);

    private static native <T> long nativeCreateSubscriptionHandle(
        long nodeHandle, Class<T> cls, String topic);

    public <T> Publisher<T> createPublisher(Class<T> cls, String topic) {
        long publisherHandle = nativeCreatePublisherHandle(this.nodeHandle, cls, topic);
        Publisher<T> publisher = new Publisher<T>(this.nodeHandle, publisherHandle);
        RCLJava.publisherReferences.add(new WeakReference<Publisher>(publisher));
        return publisher;
    }

    public <T> Subscription<T> createSubscription(Class<T> cls, String topic, Consumer<T> callback) {
        long subscriptionHandle = nativeCreateSubscriptionHandle(this.nodeHandle, cls, topic);

        Subscription<T> subscription = new Subscription<T>(this.nodeHandle, subscriptionHandle, cls, topic, callback);
        this.subscriptions.add(subscription);
        return subscription;
    }

    public List<Subscription> getSubscriptions() {
        return this.subscriptions;
    }
}

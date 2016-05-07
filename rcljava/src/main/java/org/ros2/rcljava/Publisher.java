package org.ros2.rcljava;

public class Publisher<T> {
   static {
        try {
            System.loadLibrary("rcljavaPublisher__" + RCLJava.getRMWIdentifier());
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.exit(1);
        }
    }

    private long nodeHandle;
    private long publisherHandle;

    public Publisher(long nodeHandle, long publisherHandle) {
        this.nodeHandle = nodeHandle;
        this.publisherHandle = publisherHandle;
    }

    private static native <T> void nativePublish(long publisherHandle, T msg);

    public void publish(T msg) {
        nativePublish(this.publisherHandle, msg);
    }

    private static native void nativeDispose(
        long nodeHandle, long publisherHandle);

    public void dispose() {
        nativeDispose(this.nodeHandle, this.publisherHandle);
    }
}

package io.ebinoma.rcljava;

public class Node {
   static {
        try {
            System.loadLibrary("rcljavaNode__rmw_fastrtps_cpp");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.exit(1);
        }
    }

    private long nodeHandle;

    public Node(long nodeHandle) {
        this.nodeHandle = nodeHandle;
    }

    private static native <T> long createPublisherHandle(
        long nodeHandle, Class<T> cls, String topic);

    public <T> Publisher<T> createPublisher(Class<T> cls, String topic) {
        long publisherHandle = createPublisherHandle(this.nodeHandle, cls, topic);
        Publisher<T> publisher = new Publisher<T>(publisherHandle);
        return publisher;
    }
}

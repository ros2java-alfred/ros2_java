package io.ebinoma.rcljava;

public class Publisher<T> {
   static {
        try {
            System.loadLibrary("rcljavaPublisher__rmw_opensplice_cpp");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.exit(1);
        }
    }

    private long publisherHandle;

    public Publisher(long publisherHandle) {
        this.publisherHandle = publisherHandle;
    }

    private static native <T> void nativePublish(long publisherHandle, T msg);

    public void publish(T msg) {
        nativePublish(this.publisherHandle, msg);
    }
}

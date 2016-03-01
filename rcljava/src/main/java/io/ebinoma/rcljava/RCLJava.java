package io.ebinoma.rcljava;

public class RCLJava {
   static {
        try {
            System.loadLibrary("rcljavaRCLJava__rmw_opensplice_cpp");
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.exit(1);
        }
    }

    public static native void rcljavaInit();

    private static native long createNodeHandle(String nodeName);

    public static Node createNode(String nodeName) {
        long nodeHandle = createNodeHandle(nodeName);
        Node node = new Node(nodeHandle);
        return node;
    }
}

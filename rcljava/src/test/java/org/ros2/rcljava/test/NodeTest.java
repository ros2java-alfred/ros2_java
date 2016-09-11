package org.ros2.rcljava.test;

import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;
import org.ros2.rcljava.Node;
import org.ros2.rcljava.Publisher;
import org.ros2.rcljava.QoSProfile;
import org.ros2.rcljava.RCLJava;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class NodeTest {
    private static Logger logger = Logger.getLogger(RCLJava.LOG_NAME);

    @BeforeClass
    public static void setUp() {
        logger.setLevel(Level.ALL);
        ConsoleHandler handler = new ConsoleHandler();
        handler.setFormatter(new SimpleFormatter());
        logger.addHandler(handler);
        handler.setLevel(Level.ALL);
    }

    @Test
    public void testCreateNode() {
        boolean test = true;
        Node node = null;

        RCLJava.rclJavaInit();
        try {
            node = RCLJava.createNode("testNode");
        } catch (Exception e) {
            test = false;
        }

        RCLJava.shutdown();
        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertEquals("Bad result", node, node);
    }

//    @Ignore
//    @Test
//    public void testPublisher() {
//        boolean test = true;
//        Node node = null;
//        Publisher<std_msgs.msg.String> pub = null;
//
//        RCLJava.rclJavaInit();
//        std_msgs.msg.String msg = new std_msgs.msg.String();
//        try {
//            node = RCLJava.createNode("testPublisher");
//            pub = node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class, "testChannel", QoSProfile.PROFILE_DEFAULT);
//
//        } catch (Exception e) {
//            test = false;
//        }
//
//        RCLJava.shutdown();
//        Assert.assertTrue("Expected Runtime error.", test);
//        Assert.assertNotNull("Bad result", pub);
//    }
}

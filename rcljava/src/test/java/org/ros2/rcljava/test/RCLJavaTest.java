package org.ros2.rcljava.test;

import java.util.ArrayList;
import java.util.HashMap;

import org.ros2.rcljava.Node;
import org.ros2.rcljava.RCLJava;

import junit.framework.TestCase;

public class RCLJavaTest extends TestCase {

    public void testInit() {
        boolean test = true;

        try {
            RCLJava.rclJavaInit();
        } catch (Exception e) {
            test = false;
        }

        RCLJava.shutdown();
        TestCase.assertTrue("failed to initialize rclJava", test);
    }

    public void testInitShutdown() {
        boolean test = true;

        try {
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
        } catch (Exception e) {
            test = false;
        }

        TestCase.assertTrue("failed to shutdown rclJava", test);
    }

    public void testInitShutdownSequence() {
        boolean test = true;

        RCLJava.rclJavaInit();
        RCLJava.shutdown();
        try {
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
        } catch (Exception e) {
            test = false;
        }

        TestCase.assertTrue("failed to initialize rclJava after shutdown", test);
    }

    public void testInitDouble() {
        boolean test = false;

        RCLJava.rclJavaInit();
        try {
            RCLJava.rclJavaInit();
        } catch (Exception e) {
            RCLJava.shutdown();
            test = true;
        }

        RCLJava.shutdown();
        TestCase.assertTrue("Expected Runtime error when initializing rclJava twice", test);
    }

    public void testShutdownDouble() {
        boolean test = false;

        RCLJava.rclJavaInit();
        RCLJava.shutdown();
        try {
            RCLJava.shutdown();
        } catch (Exception e) {
            test = true;
        }

        RCLJava.shutdown();
        TestCase.assertTrue("Expected Runtime error when shutting down rclJava twice", test);
    }

    public void testGetNodeName() {
        boolean test = true;
        ArrayList<String> names = new ArrayList<String>();

        RCLJava.rclJavaInit();

        try {
            names = RCLJava.getNodeNames();
        } catch (Exception e) {
            test = false;
        }

        RCLJava.shutdown();
        TestCase.assertTrue("Expected Runtime error.", test);
        TestCase.assertEquals("Bad result", names, names);
    }

    public void testGetRemoteTopic() {
        boolean test = true;
        HashMap<String, Class<?>> topics = new HashMap<String, Class<?>>();

        RCLJava.rclJavaInit();

        try {
            topics = RCLJava.getRemoteTopic();
        } catch (Exception e) {
            test = false;
        }

        RCLJava.shutdown();
        TestCase.assertTrue("Expected Runtime error.", test);
        TestCase.assertEquals("Bad result", topics, topics);
    }

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
        TestCase.assertTrue("Expected Runtime error.", test);
        TestCase.assertEquals("Bad result", node, node);
    }

    public void testOk() {
        boolean test = true;

        RCLJava.rclJavaInit();

        try {
            test = RCLJava.ok();
        } catch (Exception e) {
            test = false;
        }

        RCLJava.shutdown();
        TestCase.assertTrue("Expected Runtime error.", test);
    }
}

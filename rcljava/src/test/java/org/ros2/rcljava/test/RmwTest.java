package org.ros2.rcljava.test;

import org.junit.Ignore;
import org.junit.Test;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import org.junit.Assert;
import org.junit.BeforeClass;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.exception.NoImplementationAvailableException;

public class RmwTest {
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
    public void testRmwFastrtps() {
        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_fastrtps_cpp");
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
            RCLJava.setRMWImplementation(null);
        } catch (NoImplementationAvailableException e) {
            test = false;
        }

        Assert.assertTrue("failed to initialize rclJava with Fastrtps", test);
    }

    @Test
    public void testRmwOpensplice() {
        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_opensplice_cpp");
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
            RCLJava.setRMWImplementation(null);
        } catch (NoImplementationAvailableException e) {
            test = false;
        }

        Assert.assertTrue("failed to initialize rclJava with Opensplice", test);
    }

    @Ignore
    @Test
    public void testRmwConnext() {
        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_connext_cpp");
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
            RCLJava.setRMWImplementation(null);
        } catch (NoImplementationAvailableException e) {
            test = false;
        }

        Assert.assertTrue("failed to initialize rclJava with Connext", test);
    }

    @Ignore
    @Test
    public void testRmwConnextDynamic() {
        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_connext_dynamic_cpp");
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
            RCLJava.setRMWImplementation(null);
        } catch (NoImplementationAvailableException e) {
            test = false;
        }

        Assert.assertTrue("failed to initialize rclJava with Connext Dynamic", test);
    }

}

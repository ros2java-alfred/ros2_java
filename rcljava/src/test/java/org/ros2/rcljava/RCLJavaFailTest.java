package org.ros2.rcljava;

import org.junit.Assert;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RCLJavaFailTest {
    private static final Logger logger = LoggerFactory.getLogger(RCLJavaFailTest.class);

    @Test
    public void testInitTwice() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;

        RCLJava.rclJavaInit();

        try {
            logger.error(">>>>>>>>>>>>> Controled Fail !!! <<<<<<<<<<<<<<<");
            RCLJava.rclJavaInit();
        } catch (Exception e) {
            test = true;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error when initializing rclJava twice", test);
    }
}

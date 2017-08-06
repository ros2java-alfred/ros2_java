/* Copyright 2016 Esteve Fernandez <esteve@apache.org>
 * Copyright 2016-2017 Mickael Gaillard <mick.gaillard@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package org.ros2.rcljava;

import static org.junit.Assert.assertEquals;

import org.apache.log4j.BasicConfigurator;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;
import org.ros2.rcljava.exception.NotInitializedException;
import org.ros2.rcljava.node.NativeNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@FixMethodOrder(MethodSorters.JVM)
public class RCLJavaTest {
    private static final Logger logger = LoggerFactory.getLogger(RCLJavaTest.class);

    @BeforeClass
    public static void beforeClass() {
        BasicConfigurator.resetConfiguration();
        BasicConfigurator.configure();
    }

    @Test
    public void testInit() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            Assert.assertEquals(false, RCLJava.isInitialized());
            RCLJava.rclJavaInit();
            Assert.assertEquals(true, RCLJava.isInitialized());
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("failed to initialize rclJava", test);
    }

    @Test
    public void testInitShutdown() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue("failed to shutdown rclJava", test);
    }

    @Test
    public void testInitShutdownSequence() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        RCLJava.rclJavaInit();
        RCLJava.shutdown();

        try {
            RCLJava.rclJavaInit();
            RCLJava.shutdown();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue("failed to initialize rclJava after shutdown", test);
    }

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

    @Test
    public void testShutdownDouble() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;

        RCLJava.rclJavaInit();
        RCLJava.shutdown();

        try {
            RCLJava.shutdown();
        } catch (Exception e) {
            test = true;
        }

        Assert.assertTrue("Expected Runtime error when shutting down rclJava twice", test);
    }

    @Test
    public void testCreateNode() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        NativeNode node = null;

        RCLJava.rclJavaInit();
        try {
            node = (NativeNode)RCLJava.createNode("testNode");
            node.getName();
            node.close();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
    }

    @Test
    public void testOk() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        RCLJava.rclJavaInit();

        try {
            test = RCLJava.ok();
            Assert.assertEquals(true, RCLJava.ok());
        } catch (Exception e) {
            test = false;
        }
        Assert.assertTrue("Expected Runtime error.", test);

        try {
            RCLJava.shutdown();
            assertEquals(false, RCLJava.ok());
            test = false;
        } catch (Exception e) {  }
        Assert.assertTrue("Expected Runtime error.", test);
    }

    @Test
    public void testNotInitializedException() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;
        NativeNode node = null;

        try {
            node = (NativeNode)RCLJava.createNode("testNode");
            node.close();
        } catch (NotInitializedException e) {
            test = true;
        } catch (ExceptionInInitializerError e) {
            if (e.getCause() != null && e.getCause().getClass() == NotInitializedException.class) {
                test = true;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        Assert.assertTrue("failed not initialized exception !", test);

        try {
            node = new NativeNode("testNode");
            node.close();
//        } catch (NoClassDefFoundError e) {
//            test = true;
        } catch (NotInitializedException e) {
            test = true;
        } catch (Exception e) {
            e.printStackTrace();
        }
        Assert.assertTrue("failed not initialized exception !", test);
    }

}

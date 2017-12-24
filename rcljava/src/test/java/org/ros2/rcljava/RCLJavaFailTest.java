/* Copyright 2017 Mickael Gaillard <mick.gaillard@gmail.com>
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

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import org.ros2.rcljava.exception.NotInitializedException;
import org.ros2.rcljava.node.NativeNode;
import org.ros2.rcljava.node.Node;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RCLJavaFailTest extends AbstractRosTest {
    private static final Logger logger = LoggerFactory.getLogger(RCLJavaFailTest.class);

    @Before
    public void setUp() {
        // Disable default setUp.
    }

    @After
    public void tearDown() {
        // Disable default tearDown.
    }

    @Test
    public void testInitTwice() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;

        this.initRCLjava();

        try {
            logger.error(">>>>>>>>>>>>> Controled Fail !!! <<<<<<<<<<<<<<<");
            RCLJava.rclJavaInit();
        } catch (Exception e) {
            test = true;
        } finally {
            this.releaseRCLjava();
        }

        Assert.assertTrue("Expected Runtime error when initializing rclJava twice", test);
    }

    @Test
    @Ignore
    public void testNotInitializedException() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;
        Node node = null;

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

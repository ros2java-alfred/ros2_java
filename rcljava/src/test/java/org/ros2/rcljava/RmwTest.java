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

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import org.ros2.rcljava.exception.ImplementationAlreadyImportedException;
import org.ros2.rcljava.exception.NoImplementationAvailableException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 *
 */
public class RmwTest extends AbstractRosTest {
    private static final Logger logger = LoggerFactory.getLogger(RmwTest.class);

    @Before
    public void setUp() {
        // Disable default setUp.
    }

    @After
    public void tearDown() {
        // Disable default tearDown.
    }

    @Test
    public void testRmwFastrtps() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_fastrtps_cpp");
            this.initRCLjava();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        } finally {
            try {
                RCLJava.setRMWImplementation(null);
            } catch (Exception e) { }
        }

        Assert.assertTrue("failed to initialize rclJava with Fastrtps", test);
    }

    @Ignore
    @Test
    public void testRmwOpensplice() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_opensplice_cpp");
            this.initRCLjava();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        } finally {
            try {
                RCLJava.setRMWImplementation(null);
            } catch (Exception e) { }
        }

        Assert.assertTrue("failed to initialize rclJava with Opensplice", test);
    }

    @Ignore
    @Test
    public void testRmwConnext() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_connext_cpp");
            this.initRCLjava();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        } finally {
            try {
                RCLJava.setRMWImplementation(null);
            } catch (Exception e) { }
        }

        Assert.assertTrue("failed to initialize rclJava with Connext", test);
    }

    @Ignore
    @Test
    public void testRmwConnextDynamic() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;

        try {
            RCLJava.setRMWImplementation("rmw_connext_dynamic_cpp");
            this.initRCLjava();
            this.releaseRCLjava();
        } catch (Exception e) {
            test = false;
        } finally {
            try {
                RCLJava.setRMWImplementation(null);
            } catch (Exception e) { }
        }

        Assert.assertTrue("failed to initialize rclJava with Connext Dynamic", test);
    }

    @Test
    public void testNoImplementationAvailableException() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;

        try {
            RCLJava.setRMWImplementation("foo");
        } catch (NoImplementationAvailableException e) {
            test = true;
        } catch (ImplementationAlreadyImportedException e) {
            test = false;
        } finally {
            try {
                RCLJava.setRMWImplementation(null);
            } catch (Exception e) { }
        }

        Assert.assertTrue("failed not implementation available exception !", test);
    }

    @Test
    public void testImplementationAlreadyImportedException() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = false;

        try {
            RCLJava.setRMWImplementation("rmw_fastrtps_cpp");
            RCLJava.setRMWImplementation("rmw_fastrtps_cpp");
        } catch (ImplementationAlreadyImportedException e) {
            test = true;
        } catch (NoImplementationAvailableException e) {
            test = false;
        }  finally {
            try {
                RCLJava.setRMWImplementation(null);
            } catch (Exception e) { }
        }

        Assert.assertTrue("failed implementation already imported exception !", test);
    }
}

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

package org.ros2.rcljava.node.service;

import org.junit.Assert;
import org.junit.Test;

import org.ros2.rcljava.AbstractRosTest;
import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.qos.QoSProfile;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ServiceTest extends AbstractRosTest {
    private static final Logger logger = LoggerFactory.getLogger(ServiceTest.class);

    @Test
    public void testService() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;
        Service<rcl_interfaces.srv.GetParameters> srv = null;

        ServiceCallback<rcl_interfaces.srv.GetParameters_Request, rcl_interfaces.srv.GetParameters_Response> callback =
                new ServiceCallback<rcl_interfaces.srv.GetParameters_Request, rcl_interfaces.srv.GetParameters_Response>() {
            @Override
            public void dispatch(RMWRequestId header, rcl_interfaces.srv.GetParameters_Request request, rcl_interfaces.srv.GetParameters_Response response) { }
        };

        try {
            node = RCLJava.createNode("testSubscription");
            srv = node.<rcl_interfaces.srv.GetParameters>createService(
                    rcl_interfaces.srv.GetParameters.class,
                    "testChannel",
                    callback,
                    QoSProfile.DEFAULT);

            srv.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", srv);
    }


    @Test
    public void testClient() {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        boolean test = true;
        Node node = null;
        Client<rcl_interfaces.srv.GetParameters> clt = null;

        try {
            node = RCLJava.createNode("testClient");
            clt = node.<rcl_interfaces.srv.GetParameters>createClient(
                    rcl_interfaces.srv.GetParameters.class,
                    "testChannel",
                    QoSProfile.DEFAULT);

            clt.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", clt);
    }

}

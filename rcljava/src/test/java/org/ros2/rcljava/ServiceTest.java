package org.ros2.rcljava;

import org.junit.Assert;
import org.junit.Test;

import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.service.Client;
import org.ros2.rcljava.node.service.RMWRequestId;
import org.ros2.rcljava.node.service.Service;
import org.ros2.rcljava.node.service.ServiceCallback;
import org.ros2.rcljava.qos.QoSProfile;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ServiceTest {
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
            RCLJava.rclJavaInit();
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
        } finally {
            RCLJava.shutdown();
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
            RCLJava.rclJavaInit();
            node = RCLJava.createNode("testClient");
            clt = node.<rcl_interfaces.srv.GetParameters>createClient(
                    rcl_interfaces.srv.GetParameters.class,
                    "testChannel",
                    QoSProfile.DEFAULT);

            clt.dispose();
            node.dispose();
        } catch (Exception e) {
            test = false;
        } finally {
            RCLJava.shutdown();
        }

        Assert.assertTrue("Expected Runtime error.", test);
        Assert.assertNotNull("Bad result", clt);
    }

}

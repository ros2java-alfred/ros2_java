package org.ros2.rcljava;

import static org.junit.Assert.assertEquals;

import org.junit.Ignore;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import geometry_msgs.msg.*;

public class MessageGeometryTest extends AbstractMessageTest {
    private static final Logger logger = LoggerFactory.getLogger(MessageGeometryTest.class);

    @Test
    public final void testPubPoint32() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Point32 msg = new Point32();
        msg.setX(10f);
        msg.setY(20f);
        msg.setZ(30f);

        Point32 value = this.pubSubTest(msg);
        assertEquals(10f, value.getX(), 0.1f);
        assertEquals(20f, value.getY(), 0.1f);
        assertEquals(30f, value.getZ(), 0.1f);
    }

    @Test
    public final void testPubAccel() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Accel msg = new Accel();
        msg.getAngular().setX(10d);
        msg.getAngular().setY(20d);
        msg.getAngular().setZ(30d);
        msg.getLinear().setX(40);
        msg.getLinear().setY(50);
        msg.getLinear().setZ(60);

        Accel value = this.pubSubTest(msg);
        assertEquals(10f, value.getAngular().getX(), 0.1f);
        assertEquals(20f, value.getAngular().getY(), 0.1f);
        assertEquals(30f, value.getAngular().getZ(), 0.1f);
        assertEquals(40f, value.getLinear().getX(), 0.1f);
        assertEquals(50f, value.getLinear().getY(), 0.1f);
        assertEquals(60f, value.getLinear().getZ(), 0.1f);
    }

    @Test
    public final void testPubAccelStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        AccelStamped msg = new AccelStamped();
        msg.getAccel().getAngular().setX(10d);

        AccelStamped value = this.pubSubTest(msg);
        assertEquals(10d, value.getAccel().getAngular().getX(), 0.1d);
    }

    @Test
    @Ignore
    public final void testPubAccelWithCovariance() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        AccelWithCovariance msg = new AccelWithCovariance();
        //TODO

        AccelWithCovariance value = this.pubSubTest(msg);
    }

    @Test
    @Ignore
    public final void testPubAccelWithCovarianceStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        AccelWithCovarianceStamped msg = new AccelWithCovarianceStamped();
        //TODO

        AccelWithCovarianceStamped value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubInertia() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Inertia msg = new Inertia();
        //TODO

        Inertia value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubInertiaStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        InertiaStamped msg = new InertiaStamped();
        //TODO

        InertiaStamped value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPoint() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Point msg = new Point();
        //TODO

        Point value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPointStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        geometry_msgs.msg.PointStamped msg = new PointStamped();
        //TODO

        PointStamped value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPolygon() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Polygon msg = new Polygon();
        //TODO

        Polygon value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPolygonStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        PolygonStamped msg = new PolygonStamped();
        //TODO

        PolygonStamped value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPose() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Pose msg = new Pose();
        //TODO

        Pose value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPose2D() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        geometry_msgs.msg.Pose2D msg = new Pose2D();
        //TODO

        Pose2D value = this.pubSubTest(msg);
    }

}

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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import geometry_msgs.msg.*;

public class MessageGeometryTest extends AbstractMessageTest {
    private static final Logger logger = LoggerFactory.getLogger(MessageGeometryTest.class);

    @Test
    public final void testPubPoint32() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Point32 msg = new Point32()
                .setX(10f)
                .setY(20f)
                .setZ(30f);

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
        msg.getAccel().getAngular().setY(20d);
        msg.getAccel().getAngular().setZ(30d);

        AccelStamped value = this.pubSubTest(msg);
        assertEquals(10d, value.getAccel().getAngular().getX(), 0.1d);
        assertEquals(20d, value.getAccel().getAngular().getY(), 0.1d);
        assertEquals(30d, value.getAccel().getAngular().getZ(), 0.1d);
    }

    @Test
    //Ignore //TODO Remove after fix.
    public final void testPubAccelWithCovariance() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        AccelWithCovariance msg = new AccelWithCovariance();
        for (int i = 0; i < 36; i++) {
            msg.getCovariance().add(0D);
        }


        AccelWithCovariance value = this.pubSubTest(msg);
        assertNotNull(value);
    }

    @Test
    public final void testPubAccelWithCovarianceStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        AccelWithCovarianceStamped msg = new AccelWithCovarianceStamped();
        //msg.getAccel();

        AccelWithCovarianceStamped value = this.pubSubTest(msg);
        assertNotNull(value);
    }

    @Test
    public final void testPubInertia() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Inertia msg = new Inertia();
        msg.getCom().setX(10.0d);
        msg.getCom().setY(20.0d);
        msg.getCom().setZ(30.0d);
        msg.setIxx(40.0d);
        msg.setIxy(50.0d);
        msg.setIxz(60.0d);
        msg.setIyy(70.0d);
        msg.setIyz(80.0d);
        msg.setIzz(90.0d);
        msg.setM(100.0d);

        Inertia value = this.pubSubTest(msg);
        assertEquals(10d, value.getCom().getX(), 0.1d);
        assertEquals(20d, value.getCom().getY(), 0.1d);
        assertEquals(30d, value.getCom().getZ(), 0.1d);
        assertEquals(40d, value.getIxx(), 0.1d);
        assertEquals(50d, value.getIxy(), 0.1d);
        assertEquals(60d, value.getIxz(), 0.1d);
        assertEquals(70d, value.getIyy(), 0.1d);
        assertEquals(80d, value.getIyz(), 0.1d);
        assertEquals(90d, value.getIzz(), 0.1d);
        assertEquals(100.0d, value.getM(), 0.1d);
    }

    @Test
    public final void testPubInertiaStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        InertiaStamped msg = new InertiaStamped();
        msg.getInertia().setM(10.0d);

        InertiaStamped value = this.pubSubTest(msg);
        assertEquals(10.0d, value.getInertia().getM(), 0.1d);
    }

    @Test
    public final void testPubPoint() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Point msg = new Point();
        msg.setX(10.0d);
        msg.setY(20.0d);
        msg.setZ(30.0d);

        Point value = this.pubSubTest(msg);
        assertEquals(10.0d, value.getX(), 0.1d);
        assertEquals(20.0d, value.getY(), 0.1d);
        assertEquals(30.0d, value.getZ(), 0.1d);
    }

    @Test
    public final void testPubPointStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        PointStamped msg = new PointStamped();
        msg.getPoint().setX(10);

        PointStamped value = this.pubSubTest(msg);
        assertEquals(10.0d, value.getPoint().getX(), 0.1d);
    }

    @Test
    public final void testPubPolygon() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Polygon msg = new Polygon();
        msg.getPoints().add(new Point32());

        Polygon value = this.pubSubTest(msg);
        assertNotNull(value.getPoints().get(0));
    }

    @Test
    public final void testPubPolygonStamped() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        PolygonStamped msg = new PolygonStamped();
        msg.getPolygon().getPoints().add(new Point32());

        PolygonStamped value = this.pubSubTest(msg);
        assertNotNull(value.getPolygon().getPoints().get(0));
    }

    @Test
    public final void testPubPose() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Pose msg = new Pose();
        msg.getOrientation().setW(10.0d);
        msg.getOrientation().setX(20.0d);
        msg.getOrientation().setY(30.0d);
        msg.getOrientation().setZ(40.0d);
        msg.getPosition().setX(50.0d);
        msg.getPosition().setY(60.0d);
        msg.getPosition().setZ(70.0d);

        Pose value = this.pubSubTest(msg);
        assertEquals(10.0d, value.getOrientation().getW(), 0.1d);
        assertEquals(20.0d, value.getOrientation().getX(), 0.1d);
        assertEquals(30.0d, value.getOrientation().getY(), 0.1d);
        assertEquals(40.0d, value.getOrientation().getZ(), 0.1d);
        assertEquals(50.0d, value.getPosition().getX(), 0.1d);
        assertEquals(60.0d, value.getPosition().getY(), 0.1d);
        assertEquals(70.0d, value.getPosition().getZ(), 0.1d);
    }

    @Test
    public final void testPubPose2D() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        geometry_msgs.msg.Pose2D msg = new Pose2D();
        //TODO

        Pose2D value = this.pubSubTest(msg);
        assertNotNull(value);
    }

}

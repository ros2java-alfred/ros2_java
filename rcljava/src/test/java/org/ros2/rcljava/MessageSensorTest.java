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

import static org.junit.Assert.*;

import java.util.Arrays;

import org.junit.Ignore;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import geometry_msgs.msg.Point32;
import sensor_msgs.msg.*;


public class MessageSensorTest extends AbstractMessageTest {
    private static final Logger logger = LoggerFactory.getLogger(MessageSensorTest.class);

    @Test
    public final void testPubBatteryState() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        BatteryState msg = new BatteryState();
        msg.setCapacity(2000.0f);
//        msg.setCellVoltage(null);
        msg.setCharge(100.0f);
        msg.setCurrent(2.0f);
        msg.setDesignCapacity(2000.5f);
        msg.setLocation("test");
        msg.setPercentage(100.0f);
        msg.setPowerSupplyHealth((byte)1);
        msg.setPowerSupplyStatus((byte)2);
        msg.setPresent(true);
        msg.setSerialNumber("1234567890");
        msg.setVoltage(11.8f);

        BatteryState value = this.pubSubTest(msg);
        assertEquals(2000.0f, value.getCapacity(), 0.1f);

        assertEquals(100.0f, value.getCharge(), 0.1f);
        assertEquals(2.0f, value.getCurrent(), 0.1f);
        assertEquals(2000.5f, value.getDesignCapacity(), 0.1f);
//        assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        assertEquals(2000.0f, value.getCapacity(), 0.1f);
    }

    @Test
    @Ignore
    public final void testPubImu() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Imu msg = new Imu();
        msg.getLinearAcceleration().setX(3.0d);
        msg.getLinearAcceleration().setY(2.0d);
        msg.getLinearAcceleration().setZ(4.0d);

        // Fixed bug generate by message initialize.
        msg.getAngularVelocityCovariance().addAll(Arrays.asList(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d));
        msg.getLinearAccelerationCovariance().addAll(Arrays.asList(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d));
        msg.getOrientationCovariance().addAll(Arrays.asList(0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d, 0d));

        Imu value = this.pubSubTest(msg);
        assertEquals(3.0d, value.getLinearAcceleration().getX(), 0.1d);
    }

    @Test
    @Ignore
    public final void testPubCameraInfo() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        CameraInfo msg = new CameraInfo();
        msg.setBinningX(1);
        msg.setBinningY(2);
        msg.setDistortionModel("distortion");
        msg.setHeight(3);
        msg.setWidth(4);
        //TODO

        CameraInfo value = this.pubSubTest(msg);
        assertEquals(1, value.getBinningX());
        assertEquals(2, value.getBinningY());
        assertEquals("distortion", value.getDistortionModel());
        assertEquals(3, value.getHeight());
        assertEquals(4, value.getWidth());
    }

    @Test
    public final void testPubChannelFloat32() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        ChannelFloat32 msg = new ChannelFloat32();
        msg.setName("key");
        msg.setValues(Arrays.asList(5f));

        ChannelFloat32 value = this.pubSubTest(msg);
        assertEquals("key", value.getName());
        assertEquals(5f, value.getValues().get(0), 0.1f);
    }

    @Test
    public final void testPubCompressedImage() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        CompressedImage msg = new CompressedImage();
        msg.setFormat("format");
        //TODO

        CompressedImage value = this.pubSubTest(msg);
        assertEquals("format", value.getFormat());
    }

    @Test
    public final void testPubFluidPressure() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        FluidPressure msg = new FluidPressure();
        msg.setFluidPressure(10.0d);
        //TODO

        FluidPressure value = this.pubSubTest(msg);
        assertEquals(10.0d, value.getFluidPressure(), 0.1d);
    }

    @Test
    public final void testPubIlluminance() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Illuminance msg = new Illuminance();
        msg.setIlluminance(10d);
        msg.setVariance(20d);

        Illuminance value = this.pubSubTest(msg);
        assertEquals(10d, value.getIlluminance(), 0.1d);
        assertEquals(20d, value.getVariance(), 0.1d);
    }

    @Test
    public final void testPubImage() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Image msg = new Image();
        msg.setHeight(2);
        msg.setWidth(3);
        //TODO

        Image value = this.pubSubTest(msg);
        assertEquals(2, value.getHeight());
    }

    @Test
    public final void testPubJointState() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        JointState msg = new JointState();
        msg.getName().add("name");
        //TODO

        JointState value = this.pubSubTest(msg);
        assertEquals("name", value.getName().get(0));
    }

    @Test
    public final void testPubJoy() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Joy msg = new Joy();
        msg.getAxes().addAll(Arrays.asList(2f, 4f, 6f));
        msg.getButtons().add(10);

        Joy value = this.pubSubTest(msg);
        assertEquals(2f, value.getAxes().get(0), 0.1f);
        assertEquals(4f, value.getAxes().get(1), 0.1f);
        assertEquals(6f, value.getAxes().get(2), 0.1f);
        assertEquals((Integer)10, value.getButtons().get(0));
    }

    @Test
    public final void testPubJoyFeedback() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        JoyFeedback msg = new JoyFeedback();
        msg.setId((byte)10);
        msg.setIntensity(20f);
        msg.setType((byte)30);

        JoyFeedback value = this.pubSubTest(msg);
        assertEquals((byte)10, value.getId());
        assertEquals(20f, value.getIntensity(), 0.1f);
        assertEquals((byte)30, value.getType());
    }

    @Test
    public final void testPubJoyFeedbackArray() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        JoyFeedbackArray msg = new JoyFeedbackArray();
        //TODO

        JoyFeedbackArray value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubLaserEcho() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        LaserEcho msg = new LaserEcho();
        msg.getEchoes().addAll(Arrays.asList(21.0f, 12.0f));

        LaserEcho value = this.pubSubTest(msg);
        assertEquals(21.0f, value.getEchoes().get(0), 0.1f);
        assertEquals(12.0f, value.getEchoes().get(1), 0.1f);
    }

    @Test
    public final void testPubLaserScan() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        LaserScan msg = new LaserScan();
        msg.setAngleIncrement(10f);
        msg.setAngleMax(20f);
        msg.setAngleMin(30f);
        msg.getIntensities().add(40f);
        msg.setRangeMax(50f);
        msg.setRangeMin(60f);
        msg.getRanges().add(70f);
        msg.setScanTime(80f);
        msg.setTimeIncrement(90f);

        LaserScan value = this.pubSubTest(msg);
        assertEquals(10f, value.getAngleIncrement(), 0.1f);
        assertEquals(20f, value.getAngleMax(), 0.1f);
        assertEquals(30f, value.getAngleMin(), 0.1f);
        assertEquals(40f, value.getIntensities().get(0), 0.1f);
        assertEquals(50f, value.getRangeMax(), 0.1f);
        assertEquals(60f, value.getRangeMin(), 0.1f);
        assertEquals(70f, value.getRanges().get(0), 0.1f);
        assertEquals(80f, value.getScanTime(), 0.1f);
        assertEquals(90f, value.getTimeIncrement(), 0.1f);
    }

    @Test
    @Ignore
    public final void testPubMagneticField() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        MagneticField msg = new MagneticField();
        msg.getMagneticField().setX(10d);
        msg.getMagneticField().setY(20d);
        msg.getMagneticField().setZ(30d);
        msg.getMagneticFieldCovariance().addAll(Arrays.asList(40d));

        MagneticField value = this.pubSubTest(msg);
        assertEquals(10f, value.getMagneticField().getX(), 0.1f);
        assertEquals(20f, value.getMagneticField().getY(), 0.1f);
        assertEquals(30f, value.getMagneticField().getZ(), 0.1f);
        assertEquals(40f, value.getMagneticFieldCovariance().get(0), 0.1f);
    }

    @Test
    public final void testPubMultiDOFJointState() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        MultiDOFJointState msg = new MultiDOFJointState();
        //TODO

        MultiDOFJointState value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubMultiEchoLaserScan() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        MultiEchoLaserScan msg = new MultiEchoLaserScan();
        msg.setAngleIncrement(10f);
        msg.setAngleMax(20f);
        msg.setAngleMin(30f);
//        msg.getIntensities().add(new LaserEcho());
        msg.setRangeMax(50f);
        msg.setRangeMin(60f);
//        msg.getRanges().add(new LaserEcho());
        msg.setScanTime(80f);
        msg.setTimeIncrement(90f);

        MultiEchoLaserScan value = this.pubSubTest(msg);
        assertEquals(10f, value.getAngleIncrement(), 0.1f);
        assertEquals(20f, value.getAngleMax(), 0.1f);
        assertEquals(30f, value.getAngleMin(), 0.1f);
//        assertEquals(40f, value.getIntensities().get(0), 0.1f);
        assertEquals(50f, value.getRangeMax(), 0.1f);
        assertEquals(60f, value.getRangeMin(), 0.1f);
//        assertEquals(70f, value.getRanges().get(0), 0.1f);
        assertEquals(80f, value.getScanTime(), 0.1f);
        assertEquals(90f, value.getTimeIncrement(), 0.1f);
    }

    @Test
    @Ignore
    public final void testPubNavSatFix() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        NavSatFix msg = new NavSatFix();
        msg.setAltitude(10d);
        msg.setLatitude(20d);
        msg.setLongitude(30d);
        msg.setStatus(new NavSatStatus()); //TODO Fix this bug !! need to use type of enum.
        msg.setPositionCovarianceType((byte)50);
        msg.getPositionCovariance().add(60d);

        NavSatFix value = this.pubSubTest(msg);
        assertEquals(10f, value.getAltitude(), 0.1f);
        assertEquals(20f, value.getLatitude(), 0.1f);
        assertEquals(30f, value.getLongitude(), 0.1f);
//        assertEquals(40f, value.getStatus(), 0.1f);
        assertEquals((byte)50f, value.getPositionCovarianceType(), 0.1f);
        assertEquals(60f, value.getPositionCovariance().get(0), 0.1f);
    }

    @Test
    public final void testPubNavSatStatus() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        NavSatStatus msg = new NavSatStatus();
        //TODO

        NavSatStatus value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPointCloud() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        PointCloud msg = new PointCloud();
        msg.getChannels().add(new ChannelFloat32());
        msg.getPoints().add(new Point32());

        PointCloud value = this.pubSubTest(msg);
        assertNotNull(value.getChannels().get(0));
        assertNotNull(value.getPoints().get(0));
    }

    @Test
    public final void testPubPointCloud2() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        PointCloud2 msg = new PointCloud2();
        //TODO

        PointCloud2 value = this.pubSubTest(msg);
    }

    @Test
    public final void testPubPointField() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        PointField msg = new PointField();
        msg.setCount(10);
        msg.setDatatype((byte)20);
        msg.setName("name");
        msg.setOffset(40);

        PointField value = this.pubSubTest(msg);
        assertEquals(10, value.getCount());
        assertEquals(20, value.getDatatype());
        assertEquals("name", value.getName());
        assertEquals(40, value.getOffset());
    }

    @Test
    public final void testPubRange() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Range msg = new Range();
        msg.setFieldOfView(10f);
        msg.setMaxRange(20f);
        msg.setMinRange(30f);
        msg.setRadiationType((byte)40);
        msg.setRange(50f);

        Range value = this.pubSubTest(msg);
        assertEquals(10f, value.getFieldOfView(), 0.1f);
        assertEquals(20f, value.getMaxRange(), 0.1f);
        assertEquals(30f, value.getMinRange(), 0.1f);
        assertEquals((byte)40, value.getRadiationType(), 0.1f);
        assertEquals(50f, value.getRange(), 0.1f);
    }

    @Test
    public final void testPubRegionOfInterest() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        RegionOfInterest msg = new RegionOfInterest();
        msg.setDoRectify(true);
        msg.setHeight(20);
        msg.setWidth(30);
        msg.setXOffset(40);
        msg.setYOffset(50);

        RegionOfInterest value = this.pubSubTest(msg);
        assertEquals(true, value.getDoRectify());
        assertEquals(20, value.getHeight());
        assertEquals(30, value.getWidth());
        assertEquals(40, value.getXOffset());
        assertEquals(50, value.getYOffset());
    }

    @Test
    public final void testPubRelativeHumidity() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        RelativeHumidity msg = new RelativeHumidity();
        msg.setRelativeHumidity(10d);
        msg.setVariance(20d);

        RelativeHumidity value = this.pubSubTest(msg);
        assertEquals(10f, value.getRelativeHumidity(), 0.1f);
        assertEquals(20f, value.getVariance(), 0.1f);
    }

    @Test
    public final void testPubTemperature() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Temperature msg = new Temperature();
        msg.setTemperature(10d);
        msg.setVariance(20d);

        Temperature value = this.pubSubTest(msg);
        assertEquals(10f, value.getTemperature(), 0.1f);
        assertEquals(20f, value.getVariance(), 0.1f);
    }

    @Test
    public final void testPubTimeReference() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        TimeReference msg = new TimeReference();
        msg.setSource("source");
        msg.getTimeRef().setSec(20);
        msg.getTimeRef().setNanosec(30);

        TimeReference value = this.pubSubTest(msg);
        assertEquals("source", value.getSource());
        assertEquals(20, value.getTimeRef().getSec());
        assertEquals(30, value.getTimeRef().getNanosec());
    }
}

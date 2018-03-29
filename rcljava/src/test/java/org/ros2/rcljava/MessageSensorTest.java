/* Copyright 2017-2018 Mickael Gaillard <mick.gaillard@gmail.com>
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

import java.util.Arrays;
import java.util.concurrent.ExecutionException;

import org.junit.Assert;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import geometry_msgs.msg.Point32;
import geometry_msgs.msg.Transform;
import geometry_msgs.msg.Twist;
import geometry_msgs.msg.Wrench;
import sensor_msgs.msg.BatteryState;
import sensor_msgs.msg.CameraInfo;
import sensor_msgs.msg.ChannelFloat32;
import sensor_msgs.msg.CompressedImage;
import sensor_msgs.msg.FluidPressure;
import sensor_msgs.msg.Illuminance;
import sensor_msgs.msg.Image;
import sensor_msgs.msg.Imu;
import sensor_msgs.msg.JointState;
import sensor_msgs.msg.Joy;
import sensor_msgs.msg.JoyFeedback;
import sensor_msgs.msg.JoyFeedbackArray;
import sensor_msgs.msg.LaserEcho;
import sensor_msgs.msg.LaserScan;
import sensor_msgs.msg.MagneticField;
import sensor_msgs.msg.MultiDOFJointState;
import sensor_msgs.msg.MultiEchoLaserScan;
import sensor_msgs.msg.NavSatFix;
import sensor_msgs.msg.NavSatStatus;
import sensor_msgs.msg.PointCloud;
import sensor_msgs.msg.PointCloud2;
import sensor_msgs.msg.PointField;
import sensor_msgs.msg.Range;
import sensor_msgs.msg.RegionOfInterest;
import sensor_msgs.msg.RelativeHumidity;
import sensor_msgs.msg.Temperature;
import sensor_msgs.msg.TimeReference;

public class MessageSensorTest extends AbstractMessageTest {
    private static final Logger logger = LoggerFactory.getLogger(MessageSensorTest.class);

    @Test
    public final void testPubBatteryState() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final BatteryState msg = new BatteryState();
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

        final BatteryState value = this.pubSubTest(msg);
        Assert.assertEquals(2000.0f, value.getCapacity(), 0.1f);

        Assert.assertEquals(100.0f, value.getCharge(), 0.1f);
        Assert.assertEquals(2.0f, value.getCurrent(), 0.1f);
        Assert.assertEquals(2000.5f, value.getDesignCapacity(), 0.1f);
//        Assert.assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        Assert.assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        Assert.assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        Assert.assertEquals(2000.0f, value.getCapacity(), 0.1f);
//        Assert.assertEquals(2000.0f, value.getCapacity(), 0.1f);
    }

    @Test
    public final void testPubImu() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Imu msg = new Imu();
        msg.getLinearAcceleration().setX(3.0d);
        msg.getLinearAcceleration().setY(2.0d);
        msg.getLinearAcceleration().setZ(4.0d);

        final Imu value = this.pubSubTest(msg);
        Assert.assertEquals(3.0d, value.getLinearAcceleration().getX(), 0.1d);
    }

    @Test
    public final void testPubCameraInfo() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final CameraInfo msg = new CameraInfo();
        msg.setBinningX(10);
        msg.setBinningY(20);
        msg.getD().add(30d);
        msg.setDistortionModel("distortion");
        msg.setHeight(40);
        msg.setWidth(50);
        msg.getK().set(0, 60d);
        msg.getP().set(0, 70d);
        msg.getR().set(0, 80d);
        msg.getRoi().setDoRectify(true);

        final CameraInfo value = this.pubSubTest(msg);
        Assert.assertEquals(10, value.getBinningX());
        Assert.assertEquals(20, value.getBinningY());
        Assert.assertEquals(30d, value.getD().get(0), 0.1d);
        Assert.assertEquals("distortion", value.getDistortionModel());
        Assert.assertEquals(40, value.getHeight());
        Assert.assertEquals(50, value.getWidth());
        Assert.assertEquals(60d, value.getK().get(0), 0.1d);
        Assert.assertEquals(70d, value.getP().get(0), 0.1d);
        Assert.assertEquals(80d, value.getR().get(0), 0.1d);
        Assert.assertTrue(value.getRoi().getDoRectify());
    }

    @Test
    public final void testPubChannelFloat32() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final ChannelFloat32 msg = new ChannelFloat32();
        msg.setName("key");
        msg.setValues(Arrays.asList(5f));

        final ChannelFloat32 value = this.pubSubTest(msg);
        Assert.assertEquals("key", value.getName());
        Assert.assertEquals(5f, value.getValues().get(0), 0.1f);
    }

    @Test
    public final void testPubCompressedImage() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final CompressedImage msg = new CompressedImage();
        msg.setFormat("format");
        msg.getData().add((byte) 10);

        final CompressedImage value = this.pubSubTest(msg);
        Assert.assertEquals("format", value.getFormat());
        Assert.assertEquals((byte) 10, (byte)value.getData().get(0));
    }

    @Test
    public final void testPubFluidPressure() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final FluidPressure msg = new FluidPressure();
        msg.setFluidPressure(10.0d);
        msg.setVariance(20.0d);

        final FluidPressure value = this.pubSubTest(msg);
        Assert.assertEquals(10.0d, value.getFluidPressure(), 0.1d);
        Assert.assertEquals(20.0d, value.getVariance(), 0.1d);
    }

    @Test
    public final void testPubIlluminance() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Illuminance msg = new Illuminance();
        msg.setIlluminance(10d);
        msg.setVariance(20d);

        final Illuminance value = this.pubSubTest(msg);
        Assert.assertEquals(10d, value.getIlluminance(), 0.1d);
        Assert.assertEquals(20d, value.getVariance(), 0.1d);
    }

    @Test
    public final void testPubImage() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Image msg = new Image();
        msg.getData().add((byte)10);
        msg.setEncoding("encodingTest");
        msg.setHeight(20);
        msg.setIsBigendian((byte)30);
        msg.setStep(40);
        msg.setWidth(50);

        final Image value = this.pubSubTest(msg);
        Assert.assertEquals((byte)10, (byte)value.getData().get(0));
        Assert.assertEquals("encodingTest", value.getEncoding());
        Assert.assertEquals(20, value.getHeight());
        Assert.assertEquals((byte)30, (byte)value.getIsBigendian());
        Assert.assertEquals(40, value.getStep());
        Assert.assertEquals(50, value.getWidth());
    }

    @Test
    public final void testPubJointState() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final JointState msg = new JointState();
        msg.getEffort().add(10d);
        msg.getName().add("name");
        msg.getPosition().add(20d);
        msg.getVelocity().add(30d);

        final JointState value = this.pubSubTest(msg);
        Assert.assertEquals(10d, value.getEffort().get(0), 0.1d);
        Assert.assertEquals("name", value.getName().get(0));
        Assert.assertEquals(20d, value.getPosition().get(0), 0.1d);
        Assert.assertEquals(30d, value.getVelocity().get(0), 0.1d);
    }

    @Test
    public final void testPubJoy() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Joy msg = new Joy();
        msg.getAxes().addAll(Arrays.asList(2f, 4f, 6f));
        msg.getButtons().add(10);

        final Joy value = this.pubSubTest(msg);
        Assert.assertEquals(2f, value.getAxes().get(0), 0.1f);
        Assert.assertEquals(4f, value.getAxes().get(1), 0.1f);
        Assert.assertEquals(6f, value.getAxes().get(2), 0.1f);
        Assert.assertEquals((Integer)10, value.getButtons().get(0));
    }

    @Test
    public final void testPubJoyFeedback() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final JoyFeedback msg = new JoyFeedback();
        msg.setId((byte)10);
        msg.setIntensity(20f);
        msg.setType((byte)30);

        final JoyFeedback value = this.pubSubTest(msg);
        Assert.assertEquals((byte)10, value.getId());
        Assert.assertEquals(20f, value.getIntensity(), 0.1f);
        Assert.assertEquals((byte)30, value.getType());
    }

    @Test
    public final void testPubJoyFeedbackArray() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final JoyFeedbackArray msg = new JoyFeedbackArray();
        msg.getArray().add(new JoyFeedback());

        final JoyFeedbackArray value = this.pubSubTest(msg);
        Assert.assertNotNull(value.getArray().get(0));
    }

    @Test
    public final void testPubLaserEcho() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final LaserEcho msg = new LaserEcho();
        msg.getEchoes().addAll(Arrays.asList(21.0f, 12.0f));

        final LaserEcho value = this.pubSubTest(msg);
        Assert.assertEquals(21.0f, value.getEchoes().get(0), 0.1f);
        Assert.assertEquals(12.0f, value.getEchoes().get(1), 0.1f);
    }

    @Test
    public final void testPubLaserScan() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final LaserScan msg = new LaserScan();
        msg.setAngleIncrement(10f);
        msg.setAngleMax(20f);
        msg.setAngleMin(30f);
        msg.getIntensities().add(40f);
        msg.setRangeMax(50f);
        msg.setRangeMin(60f);
        msg.getRanges().add(70f);
        msg.setScanTime(80f);
        msg.setTimeIncrement(90f);

        final LaserScan value = this.pubSubTest(msg);
        Assert.assertEquals(10f, value.getAngleIncrement(), 0.1f);
        Assert.assertEquals(20f, value.getAngleMax(), 0.1f);
        Assert.assertEquals(30f, value.getAngleMin(), 0.1f);
        Assert.assertEquals(40f, value.getIntensities().get(0), 0.1f);
        Assert.assertEquals(50f, value.getRangeMax(), 0.1f);
        Assert.assertEquals(60f, value.getRangeMin(), 0.1f);
        Assert.assertEquals(70f, value.getRanges().get(0), 0.1f);
        Assert.assertEquals(80f, value.getScanTime(), 0.1f);
        Assert.assertEquals(90f, value.getTimeIncrement(), 0.1f);
    }

    @Test
    public final void testPubMagneticField() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final MagneticField msg = new MagneticField();
        msg.getMagneticField().setX(10d);
        msg.getMagneticField().setY(20d);
        msg.getMagneticField().setZ(30d);
        msg.getMagneticFieldCovariance().set(0, 40d);

        final MagneticField value = this.pubSubTest(msg);
        Assert.assertEquals(10f, value.getMagneticField().getX(), 0.1f);
        Assert.assertEquals(20f, value.getMagneticField().getY(), 0.1f);
        Assert.assertEquals(30f, value.getMagneticField().getZ(), 0.1f);
        Assert.assertEquals(40f, value.getMagneticFieldCovariance().get(0), 0.1f);
    }

    @Test
    public final void testPubMultiDOFJointState() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final MultiDOFJointState msg = new MultiDOFJointState();
        msg.getJointNames().add("test");
        msg.getTransforms().add(new Transform());
        msg.getTwist().add(new Twist());
        msg.getWrench().add(new Wrench());

        final MultiDOFJointState value = this.pubSubTest(msg);
        Assert.assertNotNull(value.getJointNames().get(0));
        Assert.assertNotNull(value.getTransforms().get(0));
        Assert.assertNotNull(value.getTwist().get(0));
        Assert.assertNotNull(value.getWrench().get(0));
    }

    @Test
    public final void testPubMultiEchoLaserScan() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final MultiEchoLaserScan msg = new MultiEchoLaserScan();
        msg.setAngleIncrement(10f);
        msg.setAngleMax(20f);
        msg.setAngleMin(30f);
//        msg.getIntensities().add(new LaserEcho());
        msg.setRangeMax(50f);
        msg.setRangeMin(60f);
//        msg.getRanges().add(new LaserEcho());
        msg.setScanTime(80f);
        msg.setTimeIncrement(90f);

        final MultiEchoLaserScan value = this.pubSubTest(msg);
        Assert.assertEquals(10f, value.getAngleIncrement(), 0.1f);
        Assert.assertEquals(20f, value.getAngleMax(), 0.1f);
        Assert.assertEquals(30f, value.getAngleMin(), 0.1f);
//        Assert.assertEquals(40f, value.getIntensities().get(0), 0.1f);
        Assert.assertEquals(50f, value.getRangeMax(), 0.1f);
        Assert.assertEquals(60f, value.getRangeMin(), 0.1f);
//        Assert.assertEquals(70f, value.getRanges().get(0), 0.1f);
        Assert.assertEquals(80f, value.getScanTime(), 0.1f);
        Assert.assertEquals(90f, value.getTimeIncrement(), 0.1f);
    }

    @Test
    public final void testPubNavSatFix() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final NavSatFix msg = new NavSatFix();
        msg.setAltitude(10d);
        msg.setLatitude(20d);
        msg.setLongitude(30d);
        msg.setStatus(new NavSatStatus()); //TODO Fix this bug !! need to use type of enum.
        msg.setPositionCovarianceType((byte)50);
        msg.getPositionCovariance().set(0, 60d);

        final NavSatFix value = this.pubSubTest(msg);
        Assert.assertEquals(10f, value.getAltitude(), 0.1f);
        Assert.assertEquals(20f, value.getLatitude(), 0.1f);
        Assert.assertEquals(30f, value.getLongitude(), 0.1f);
//        Assert.assertEquals(40f, value.getStatus(), 0.1f);
        Assert.assertEquals((byte)50f, value.getPositionCovarianceType(), 0.1f);
        Assert.assertEquals(60f, value.getPositionCovariance().get(0), 0.1f);
    }

    @Test
    public final void testPubNavSatStatus() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final NavSatStatus msg = new NavSatStatus();
        msg.setService((short) 10);
        msg.setStatus((byte)20);

        final NavSatStatus value = this.pubSubTest(msg);
        Assert.assertEquals((short)10, value.getService());
        Assert.assertEquals((byte)20, value.getStatus());
    }

    @Test
    public final void testPubPointCloud() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final PointCloud msg = new PointCloud();
        msg.getChannels().add(new ChannelFloat32());
        msg.getPoints().add(new Point32());

        final PointCloud value = this.pubSubTest(msg);
        Assert.assertNotNull(value.getChannels().get(0));
        Assert.assertNotNull(value.getPoints().get(0));
    }

    @Test
    public final void testPubPointCloud2() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final PointCloud2 msg = new PointCloud2();
        msg.getData().add((byte) 10);
        msg.getFields().add(new PointField());
        msg.setHeight(20);
        msg.setIsBigendian(true);
        msg.setIsDense(true);
        msg.setPointStep(30);
        msg.setRowStep(40);
        msg.setWidth(50);

        final PointCloud2 value = this.pubSubTest(msg);
        Assert.assertEquals((byte)10, (byte)value.getData().get(0));
        Assert.assertNotNull(value.getFields().get(0));
        Assert.assertEquals(20, value.getHeight());
        Assert.assertTrue(value.getIsBigendian());
        Assert.assertTrue(value.getIsDense());
        Assert.assertEquals(30, value.getPointStep());
        Assert.assertEquals(40, value.getRowStep());
        Assert.assertEquals(50, value.getWidth());
    }

    @Test
    public final void testPubPointField() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final PointField msg = new PointField();
        msg.setCount(10);
        msg.setDatatype((byte)20);
        msg.setName("name");
        msg.setOffset(40);

        final PointField value = this.pubSubTest(msg);
        Assert.assertEquals(10, value.getCount());
        Assert.assertEquals(20, value.getDatatype());
        Assert.assertEquals("name", value.getName());
        Assert.assertEquals(40, value.getOffset());
    }

    @Test
    public final void testPubRange() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Range msg = new Range();
        msg.setFieldOfView(10f);
        msg.setMaxRange(20f);
        msg.setMinRange(30f);
        msg.setRadiationType((byte)40);
        msg.setRange(50f);

        final Range value = this.pubSubTest(msg);
        Assert.assertEquals(10f, value.getFieldOfView(), 0.1f);
        Assert.assertEquals(20f, value.getMaxRange(), 0.1f);
        Assert.assertEquals(30f, value.getMinRange(), 0.1f);
        Assert.assertEquals((byte)40, value.getRadiationType(), 0.1f);
        Assert.assertEquals(50f, value.getRange(), 0.1f);
    }

    @Test
    public final void testPubRegionOfInterest() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final RegionOfInterest msg = new RegionOfInterest();
        msg.setDoRectify(true);
        msg.setHeight(20);
        msg.setWidth(30);
        msg.setXOffset(40);
        msg.setYOffset(50);

        final RegionOfInterest value = this.pubSubTest(msg);
        Assert.assertEquals(true, value.getDoRectify());
        Assert.assertEquals(20, value.getHeight());
        Assert.assertEquals(30, value.getWidth());
        Assert.assertEquals(40, value.getXOffset());
        Assert.assertEquals(50, value.getYOffset());
    }

    @Test
    public final void testPubRelativeHumidity() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final RelativeHumidity msg = new RelativeHumidity();
        msg.setRelativeHumidity(10d);
        msg.setVariance(20d);

        final RelativeHumidity value = this.pubSubTest(msg);
        Assert.assertEquals(10f, value.getRelativeHumidity(), 0.1f);
        Assert.assertEquals(20f, value.getVariance(), 0.1f);
    }

    @Test
    public final void testPubTemperature() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Temperature msg = new Temperature();
        msg.setTemperature(10d);
        msg.setVariance(20d);

        final Temperature value = this.pubSubTest(msg);
        Assert.assertEquals(10f, value.getTemperature(), 0.1f);
        Assert.assertEquals(20f, value.getVariance(), 0.1f);
    }

    @Test
    public final void testPubTimeReference() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final TimeReference msg = new TimeReference();
        msg.setSource("source");
        msg.getTimeRef().setSec(20);
        msg.getTimeRef().setNanosec(30);

        final TimeReference value = this.pubSubTest(msg);
        Assert.assertEquals("source", value.getSource());
        Assert.assertEquals(20, value.getTimeRef().getSec());
        Assert.assertEquals(30, value.getTimeRef().getNanosec());
    }
}

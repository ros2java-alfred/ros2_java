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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.ExecutionException;

import org.junit.Assert;
import org.junit.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import std_msgs.msg.Bool;
import std_msgs.msg.Byte;
import std_msgs.msg.Char;
import std_msgs.msg.ColorRGBA;
import std_msgs.msg.Empty;
import std_msgs.msg.Float32;
import std_msgs.msg.Float32MultiArray;
import std_msgs.msg.Float64;
import std_msgs.msg.Float64MultiArray;
import std_msgs.msg.Header;
import std_msgs.msg.Int16;
import std_msgs.msg.Int32;
import std_msgs.msg.Int64;
import std_msgs.msg.Int8;
import std_msgs.msg.MultiArrayDimension;
import std_msgs.msg.MultiArrayLayout;
import std_msgs.msg.String;
import std_msgs.msg.UInt16;
import std_msgs.msg.UInt32;
import std_msgs.msg.UInt64;
import std_msgs.msg.UInt8;

public class MessageStdTest extends AbstractMessageTest {
    private static final Logger logger = LoggerFactory.getLogger(MessageStdTest.class);

    @Test
    public final void testPubBool() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Bool msg = new Bool();
        msg.setData(true);

        final Bool value = this.pubSubTest(msg);
        Assert.assertEquals(true, value.getData());
    }

    @Test
    public final void testPubByte() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Byte msg = new Byte();
        msg.setData((byte) 3);

        final Byte value = this.pubSubTest(msg);
        Assert.assertEquals((byte) 3, value.getData());
    }

    @Test
    public final void testPubChar() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Char msg = new Char();
        msg.setData('c');

        final Char value = this.pubSubTest(msg);
        Assert.assertEquals('c', value.getData());
    }

    @Test
    public final void testPubColorRGBA() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final ColorRGBA msg = new ColorRGBA();
        msg.setR(1.0F);
        msg.setB(1.1F);
        msg.setG(1.2F);
        msg.setA(1.3F);

        final ColorRGBA value = this.pubSubTest(msg);
        Assert.assertEquals(1.0F, value.getR(), 0.1);
        Assert.assertEquals(1.1F, value.getB(), 0.1);
        Assert.assertEquals(1.2F, value.getG(), 0.1);
        Assert.assertEquals(1.3F, value.getA(), 0.1);
    }

    @Test
    public final void testPubEmpty() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Empty msg = new Empty();

        final Empty value = this.pubSubTest(msg);
        Assert.assertNotEquals(null, value);
    }

    @Test
    public final void testPubFloat32() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Float32 msg = new Float32();
        msg.setData(123.45f);

        final Float32 value = this.pubSubTest(msg);
        Assert.assertEquals(123.45f, value.getData(), 0.1);
    }

    @Test
    public final void testPubFloat64() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Float64 msg = new Float64();
        msg.setData(123.45f);

        final Float64 value = this.pubSubTest(msg);
        Assert.assertEquals(123.45f, value.getData(), 0.1);
    }

    @Test
    public final void testPubUInt8() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final UInt8 msg = new UInt8();
        msg.setData((byte) 12345);

        final UInt8 value = this.pubSubTest(msg);
        Assert.assertEquals((byte)12345, value.getData());
    }

    @Test
    public final void testPubUInt16() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final UInt16 msg = new UInt16();
        msg.setData((short) 12345);

        final UInt16 value = this.pubSubTest(msg);
        Assert.assertEquals((short)12345, value.getData());
    }

    @Test
    public final void testPubUInt32() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final UInt32 msg = new UInt32();
        msg.setData(12345);

        final UInt32 value = this.pubSubTest(msg);
        Assert.assertEquals(12345, value.getData());
    }

    @Test
    public final void testPubUInt64() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final UInt64 msg = new UInt64();
        msg.setData(12345L);

        final UInt64 value = this.pubSubTest(msg);
        Assert.assertEquals(12345L, value.getData());
    }

    @Test
    public final void testPubInt8() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Int8 msg = new Int8();
        msg.setData((byte) -12345);

        final Int8 value = this.pubSubTest(msg);
        Assert.assertEquals((byte) -12345, value.getData());
    }

    @Test
    public final void testPubInt16() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Int16 msg = new Int16();
        msg.setData((short) -12345);

        final Int16 value = this.pubSubTest(msg);
        Assert.assertEquals((short) -12345, value.getData());
    }

    @Test
    public final void testPubInt32() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Int32 msg = new Int32();
        msg.setData(-12345);

        final Int32 value = this.pubSubTest(msg);
        Assert.assertEquals(-12345, value.getData());
    }

    @Test
    public final void testPubInt64() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Int64 msg = new Int64();
        msg.setData(12345L);

        final Int64 value = this.pubSubTest(msg);
        Assert.assertEquals(12345L, value.getData());
    }

    @Test
    public final void testPubString() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final String msg = new std_msgs.msg.String();
        msg.setData("Hello");

        final String value = this.pubSubTest(msg);
        Assert.assertEquals("Hello", value.getData());
    }

    @Test
    public final void testPubHeader() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Header msg = new Header();
        msg.getStamp().setSec(21);
        msg.getStamp().setNanosec(12);
        msg.setFrameId("test21");

        final Header value = this.pubSubTest(msg);
        Assert.assertEquals(21, value.getStamp().getSec());
        Assert.assertEquals(12, value.getStamp().getNanosec());
        Assert.assertEquals("test21", value.getFrameId());
    }

    @Test
    public final void testPubMultiArrayDimension() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final MultiArrayDimension msg = new MultiArrayDimension();
        msg.setLabel("Hello");
        msg.setSize(3);
        msg.setStride(2);

        final MultiArrayDimension value = this.pubSubTest(msg);
        Assert.assertEquals("Hello", value.getLabel());
        Assert.assertEquals(3, value.getSize());
        Assert.assertEquals(2, value.getStride());
    }

    @Test
    public final void testPubMultiArrayLayout() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final List<MultiArrayDimension> subMsgs = new ArrayList<MultiArrayDimension>();
        final MultiArrayDimension subMsg = new MultiArrayDimension();
        subMsg.setLabel("Hello");
        subMsg.setSize(3);
        subMsg.setStride(2);
        subMsgs.add(subMsg);

        final MultiArrayLayout msg = new MultiArrayLayout();
        msg.setDim(subMsgs);
        msg.setDataOffset(1);

        final MultiArrayLayout value = this.pubSubTest(msg);
        Assert.assertEquals("Hello", value.getDim().get(0).getLabel());
        Assert.assertEquals(3, value.getDim().get(0).getSize());
        Assert.assertEquals(2, value.getDim().get(0).getStride());
        Assert.assertEquals(1, value.getDataOffset());
    }

    @Test
    public final void testPubFloat32MultiArray() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Float32MultiArray msg = new Float32MultiArray();

        // Collection Data
        final Collection<Float> subMsgs = new ArrayList<Float>();
        subMsgs.add(21.34f);
        msg.setData(subMsgs);

        // Layout Data
        final List<MultiArrayDimension> sub1Msgs = new ArrayList<MultiArrayDimension>();
        final MultiArrayDimension subMsg = new MultiArrayDimension();
        subMsg.setLabel("Hello");
        subMsg.setSize(3);
        subMsg.setStride(2);
        sub1Msgs.add(subMsg);

        final MultiArrayLayout sub2Msgs = new MultiArrayLayout();
        sub2Msgs.setDim(sub1Msgs);
        sub2Msgs.setDataOffset(1);
        msg.setLayout(sub2Msgs);

        final Float32MultiArray value = this.pubSubTest(msg);
        Assert.assertEquals(21.34f, value.getData().get(0), 0.1);
        Assert.assertNotEquals(null, value.getLayout());
        Assert.assertEquals(1, value.getLayout().getDataOffset());
    }

    @Test
    public final void testPubFloat64MultiArray() throws InterruptedException, ExecutionException {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        final Float64MultiArray msg = new Float64MultiArray();

        // Collection Data
        final Collection<Double> subMsgs = new ArrayList<Double>();
        subMsgs.add(21.34d);
        msg.setData(subMsgs);

        // Layout Data
        final List<MultiArrayDimension> sub1Msgs = new ArrayList<MultiArrayDimension>();
        final MultiArrayDimension subMsg = new MultiArrayDimension();
        subMsg.setLabel("Hello");
        subMsg.setSize(3);
        subMsg.setStride(2);
        sub1Msgs.add(subMsg);

        final MultiArrayLayout sub2Msgs = new MultiArrayLayout();
        sub2Msgs.setDim(sub1Msgs);
        sub2Msgs.setDataOffset(1);
        msg.setLayout(sub2Msgs);

        final Float64MultiArray value = this.pubSubTest(msg);
        Assert.assertEquals(21.34d, value.getData().get(0), 0.1);
        Assert.assertNotEquals(null, value.getLayout());
        Assert.assertEquals(1, value.getLayout().getDataOffset());
    }

}

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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

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
    public final void testPubBool() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Bool msg = new Bool();
        msg.setData(true);

        Bool value = this.pubSubTest(msg);
        assertEquals(true, value.getData());
    }

    @Test
    public final void testPubByte() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Byte msg = new Byte();
        msg.setData((byte) 3);

        Byte value = this.pubSubTest(msg);
        assertEquals((byte) 3, value.getData());
    }

    @Test
    public final void testPubChar() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Char msg = new Char();
        msg.setData('c');

        Char value = this.pubSubTest(msg);
        assertEquals('c', value.getData());
    }

    @Test
    public final void testPubColorRGBA() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        ColorRGBA msg = new ColorRGBA();
        msg.setR(1.0F);
        msg.setB(1.1F);
        msg.setG(1.2F);
        msg.setA(1.3F);

        ColorRGBA value = this.pubSubTest(msg);
        assertEquals(1.0F, value.getR(), 0.1);
        assertEquals(1.1F, value.getB(), 0.1);
        assertEquals(1.2F, value.getG(), 0.1);
        assertEquals(1.3F, value.getA(), 0.1);
    }

    @Test
    public final void testPubEmpty() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Empty msg = new Empty();

        Empty value = this.pubSubTest(msg);
        assertNotEquals(null, value);
    }

    @Test
    public final void testPubFloat32() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Float32 msg = new Float32();
        msg.setData(123.45f);

        Float32 value = this.pubSubTest(msg);
        assertEquals(123.45f, value.getData(), 0.1);
    }

    @Test
    public final void testPubFloat64() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Float64 msg = new Float64();
        msg.setData(123.45f);

        Float64 value = this.pubSubTest(msg);
        assertEquals(123.45f, value.getData(), 0.1);
    }

    @Test
    public final void testPubUInt8() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        UInt8 msg = new UInt8();
        msg.setData((byte) 12345);

        UInt8 value = this.pubSubTest(msg);
        assertEquals((byte)12345, value.getData());
    }

    @Test
    public final void testPubUInt16() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        UInt16 msg = new UInt16();
        msg.setData((short) 12345);

        UInt16 value = this.pubSubTest(msg);
        assertEquals((short)12345, value.getData());
    }

    @Test
    public final void testPubUInt32() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        UInt32 msg = new UInt32();
        msg.setData(12345);

        UInt32 value = this.pubSubTest(msg);
        assertEquals(12345, value.getData());
    }

    @Test
    public final void testPubUInt64() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        UInt64 msg = new UInt64();
        msg.setData(12345L);

        UInt64 value = this.pubSubTest(msg);
        assertEquals(12345L, value.getData());
    }

    @Test
    public final void testPubInt8() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Int8 msg = new Int8();
        msg.setData((byte) -12345);

        Int8 value = this.pubSubTest(msg);
        assertEquals((byte) -12345, value.getData());
    }

    @Test
    public final void testPubInt16() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Int16 msg = new Int16();
        msg.setData((short) -12345);

        Int16 value = this.pubSubTest(msg);
        assertEquals((short) -12345, value.getData());
    }

    @Test
    public final void testPubInt32() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Int32 msg = new Int32();
        msg.setData(-12345);

        Int32 value = this.pubSubTest(msg);
        assertEquals(-12345, value.getData());
    }

    @Test
    public final void testPubInt64() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Int64 msg = new Int64();
        msg.setData(12345L);

        Int64 value = this.pubSubTest(msg);
        assertEquals(12345L, value.getData());
    }

    @Test
    public final void testPubString() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        String msg = new std_msgs.msg.String();
        msg.setData("Hello");

        String value = this.pubSubTest(msg);
        assertEquals("Hello", value.getData());
    }

    @Test
    public final void testPubHeader() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Header msg = new Header();
        msg.getStamp().setSec(21);
        msg.getStamp().setNanosec(12);
        msg.setFrameId("test21");

        Header value = this.pubSubTest(msg);
        assertEquals(21, value.getStamp().getSec());
        assertEquals(12, value.getStamp().getNanosec());
        assertEquals("test21", value.getFrameId());
    }

    @Test
    public final void testPubMultiArrayDimension() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        MultiArrayDimension msg = new MultiArrayDimension();
        msg.setLabel("Hello");
        msg.setSize(3);
        msg.setStride(2);

        MultiArrayDimension value = this.pubSubTest(msg);
        assertEquals("Hello", value.getLabel());
        assertEquals(3, value.getSize());
        assertEquals(2, value.getStride());
    }

    @Test
    public final void testPubMultiArrayLayout() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        List<MultiArrayDimension> subMsgs = new ArrayList<MultiArrayDimension>();
        MultiArrayDimension subMsg = new MultiArrayDimension();
        subMsg.setLabel("Hello");
        subMsg.setSize(3);
        subMsg.setStride(2);
        subMsgs.add(subMsg);

        MultiArrayLayout msg = new MultiArrayLayout();
        msg.setDim(subMsgs);
        msg.setDataOffset(1);

        MultiArrayLayout value = this.pubSubTest(msg);
        assertEquals("Hello", value.getDim().get(0).getLabel());
        assertEquals(3, value.getDim().get(0).getSize());
        assertEquals(2, value.getDim().get(0).getStride());
        assertEquals(1, value.getDataOffset());
    }

    @Test
    public final void testPubFloat32MultiArray() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Float32MultiArray msg = new Float32MultiArray();

        // Collection Data
        Collection<Float> subMsgs = new ArrayList<Float>();
        subMsgs.add(21.34f);
        msg.setData(subMsgs);

        // Layout Data
        List<MultiArrayDimension> sub1Msgs = new ArrayList<MultiArrayDimension>();
        MultiArrayDimension subMsg = new MultiArrayDimension();
        subMsg.setLabel("Hello");
        subMsg.setSize(3);
        subMsg.setStride(2);
        sub1Msgs.add(subMsg);

        MultiArrayLayout sub2Msgs = new MultiArrayLayout();
        sub2Msgs.setDim(sub1Msgs);
        sub2Msgs.setDataOffset(1);
        msg.setLayout(sub2Msgs);

        Float32MultiArray value = this.pubSubTest(msg);
        assertEquals(21.34f, value.getData().get(0), 0.1);
        assertNotEquals(null, value.getLayout());
        assertEquals(1, value.getLayout().getDataOffset());
    }

    @Test
    public final void testPubFloat64MultiArray() throws Exception {
        logger.debug(new Object(){}.getClass().getEnclosingMethod().getName());

        Float64MultiArray msg = new Float64MultiArray();

        // Collection Data
        Collection<Double> subMsgs = new ArrayList<Double>();
        subMsgs.add(21.34d);
        msg.setData(subMsgs);

        // Layout Data
        List<MultiArrayDimension> sub1Msgs = new ArrayList<MultiArrayDimension>();
        MultiArrayDimension subMsg = new MultiArrayDimension();
        subMsg.setLabel("Hello");
        subMsg.setSize(3);
        subMsg.setStride(2);
        sub1Msgs.add(subMsg);

        MultiArrayLayout sub2Msgs = new MultiArrayLayout();
        sub2Msgs.setDim(sub1Msgs);
        sub2Msgs.setDataOffset(1);
        msg.setLayout(sub2Msgs);

        Float64MultiArray value = this.pubSubTest(msg);
        assertEquals(21.34d, value.getData().get(0), 0.1);
        assertNotEquals(null, value.getLayout());
        assertEquals(1, value.getLayout().getDataOffset());
    }

}

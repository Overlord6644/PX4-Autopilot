/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Unit tests for the FBUS (F.Port v2) master protocol core.
 *
 * The uplink test vector 08 AC 10 00 68 23 4A 2A 00 43 is a real frame
 * captured on the Teensy bench from an Xact W5651 (3.5 A, 7.4 V, 42 degC).
 */

#include <gtest/gtest.h>
#include <string.h>

#include "FbusProtocol.hpp"

namespace
{

// Real captured Xact uplink: phyID 0x0C (+check bits = 0xAC), prim DATA,
// appId 0x6800 (servoId 0), current 3.5 A, voltage 7.4 V, temp 42 degC.
const uint8_t kCapturedUplink[10] = {0x08, 0xAC, 0x10, 0x00, 0x68, 0x23, 0x4A, 0x2A, 0x00, 0x43};

// Build a CRC-valid 10-byte uplink frame.
void buildUplink(uint8_t out[10], uint8_t phy_id_raw, uint8_t prim, uint16_t app_id,
		 uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
	out[0] = 0x08;
	out[1] = FbusProtocol::phyIdWithCheckBits(phy_id_raw);
	out[2] = prim;
	out[3] = (uint8_t)(app_id & 0xFF);
	out[4] = (uint8_t)(app_id >> 8);
	out[5] = d0;
	out[6] = d1;
	out[7] = d2;
	out[8] = d3;
	out[9] = FbusProtocol::frskyCheckSum(&out[1], 8);
}

// Decode one 11-bit channel from the LSB-first packed control payload.
uint16_t unpackChannel(const uint8_t *channel_bytes, uint8_t ch)
{
	const uint32_t bitpos = (uint32_t)ch * 11;
	const uint32_t byte_index = bitpos >> 3;
	const uint32_t bit_index = bitpos & 7;

	uint32_t value = channel_bytes[byte_index] >> bit_index;
	value |= (uint32_t)channel_bytes[byte_index + 1] << (8 - bit_index);

	if (bit_index > 5) {
		value |= (uint32_t)channel_bytes[byte_index + 2] << (16 - bit_index);
	}

	return (uint16_t)(value & 0x7FF);
}

} // namespace

TEST(FbusProtocol, FrskyChecksumMatchesCapturedFrame)
{
	// CRC covers bytes 1..8 (phyID through data)
	EXPECT_EQ(FbusProtocol::frskyCheckSum(&kCapturedUplink[1], 8), kCapturedUplink[9]);
}

TEST(FbusProtocol, PhyIdCheckBits)
{
	EXPECT_EQ(FbusProtocol::phyIdWithCheckBits(0x0C), 0xAC);
	EXPECT_EQ(FbusProtocol::stripPhyIdCheckBits(0xAC), 0x0C);

	// All 27 valid IDs must round-trip
	for (uint8_t id = 0; id <= 0x1A; id++) {
		EXPECT_EQ(FbusProtocol::stripPhyIdCheckBits(FbusProtocol::phyIdWithCheckBits(id)), (int8_t)id);
	}

	// Corrupted check bits must be rejected
	EXPECT_EQ(FbusProtocol::stripPhyIdCheckBits(0xAC ^ 0x80), -1);
}

TEST(FbusProtocol, MicrosecondsToSbus)
{
	// Standard FrSky SBUS line: us = sbus / 1.6 + 880
	EXPECT_EQ(FbusProtocol::microsecondsToSbus(1000), 192);
	EXPECT_EQ(FbusProtocol::microsecondsToSbus(1500), 992);
	EXPECT_EQ(FbusProtocol::microsecondsToSbus(1520), 1024);
	EXPECT_EQ(FbusProtocol::microsecondsToSbus(2000), 1792);

	// Clamped outside the representable range
	EXPECT_EQ(FbusProtocol::microsecondsToSbus(500), 192);
	EXPECT_EQ(FbusProtocol::microsecondsToSbus(2500), 1792);
}

TEST(FbusProtocol, BuildsValidControlAndDownlinkFrame)
{
	FbusProtocol fbus;
	uint64_t now = 1000000;
	fbus.reset(now);

	fbus.setChannel(0, 2000);
	fbus.setChannel(1, 1000);
	fbus.setChannel(2, 1520);

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	ASSERT_EQ(fbus.update(now, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);

	// Control frame header + CRC over type..rssi
	EXPECT_EQ(frame[0], 0x18);
	EXPECT_EQ(frame[1], 0xFF);
	EXPECT_EQ(FbusProtocol::frskyCheckSum(&frame[1], 25), frame[26]);

	// Channel packing round-trip
	EXPECT_EQ(unpackChannel(&frame[2], 0), 1792);
	EXPECT_EQ(unpackChannel(&frame[2], 1), 192);
	EXPECT_EQ(unpackChannel(&frame[2], 2), 1024);
	EXPECT_EQ(unpackChannel(&frame[2], 3), 992);	// untouched channel = neutral 1500 us

	// Downlink: first slot is a discovery poll for physical ID 0
	EXPECT_EQ(frame[27], 0x08);
	EXPECT_EQ(frame[28], FbusProtocol::phyIdWithCheckBits(0));
	EXPECT_EQ(frame[29], FbusProtocol::FRAME_ID_DATA);
	EXPECT_EQ(FbusProtocol::frskyCheckSum(&frame[28], 8), frame[36]);
}

TEST(FbusProtocol, PollResponseWindowAndTimeout)
{
	FbusProtocol fbus;
	uint64_t now = 0;
	fbus.reset(now);

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	ASSERT_EQ(fbus.update(now, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);

	// Inside the response window nothing is transmitted, even past the frame period
	EXPECT_EQ(fbus.update(now + 2500, frame, sizeof(frame)), 0u);
	EXPECT_EQ(fbus.responseTimeouts(), 0u);

	// Past 3000 us the poll times out and the next frame goes out
	EXPECT_EQ(fbus.update(now + 3100, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);
	EXPECT_EQ(fbus.responseTimeouts(), 1u);
}

TEST(FbusProtocol, DiscoveryAndTelemetryFromCapturedFrame)
{
	FbusProtocol fbus;
	uint64_t now = 0;
	fbus.reset(now);

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	ASSERT_EQ(fbus.update(now, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);

	// Servo answers the discovery poll with the captured telemetry frame
	fbus.processRx(kCapturedUplink, sizeof(kCapturedUplink), now + 800);

	// No echo preceded the reply: the port is classified echo-free
	EXPECT_EQ(fbus.echoState(), FbusProtocol::EchoState::Absent);
	EXPECT_EQ(fbus.framesReceived(), 1u);
	EXPECT_EQ(fbus.crcErrors(), 0u);

	// Sensor registered during scan
	ASSERT_EQ(fbus.sensorCount(), 1);
	EXPECT_EQ(fbus.sensorId(0), 0x0C);

	// Telemetry decoded: 3.5 A, 7.4 V, 42 degC on servoId 0
	FbusProtocol::ServoTelemetry telem{};
	ASSERT_TRUE(fbus.getServoTelemetry(0, now + 1000, telem));
	EXPECT_FLOAT_EQ(telem.current_a, 3.5f);
	EXPECT_FLOAT_EQ(telem.voltage_v, 7.4f);
	EXPECT_EQ(telem.temperature_c, 42);

	// Stale after 1 s without updates
	EXPECT_FALSE(fbus.getServoTelemetry(0, now + 800 + 1100000, telem));

	// Unknown servoId rejected
	EXPECT_FALSE(fbus.getServoTelemetry(7, now + 1000, telem));
}

TEST(FbusProtocol, EchoFilterDetectsAndConsumesEcho)
{
	FbusProtocol fbus;
	uint64_t now = 0;
	fbus.reset(now);

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	ASSERT_EQ(fbus.update(now, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);

	// The UART loops the whole transmitted frame back
	fbus.processRx(frame, sizeof(frame), now + 850);
	EXPECT_EQ(fbus.echoState(), FbusProtocol::EchoState::Present);
	EXPECT_EQ(fbus.framesReceived(), 0u);
	EXPECT_EQ(fbus.crcErrors(), 0u);
	EXPECT_EQ(fbus.rawRxBytes(), FbusProtocol::FRAME_SIZE);

	// The real reply after the echo still parses
	fbus.processRx(kCapturedUplink, sizeof(kCapturedUplink), now + 900);
	EXPECT_EQ(fbus.framesReceived(), 1u);
	ASSERT_EQ(fbus.sensorCount(), 1);

	// Next cycle: echo consumed again before the reply
	ASSERT_EQ(fbus.update(now + 3000, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);
	fbus.processRx(frame, sizeof(frame), now + 3800);
	fbus.processRx(kCapturedUplink, sizeof(kCapturedUplink), now + 3900);
	EXPECT_EQ(fbus.framesReceived(), 2u);
}

TEST(FbusProtocol, SecondFrameDownlinkIsNullBetweenPolls)
{
	FbusProtocol fbus;
	uint64_t now = 0;
	fbus.reset(now);

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	ASSERT_EQ(fbus.update(now, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);
	fbus.processRx(kCapturedUplink, sizeof(kCapturedUplink), now + 800);

	// 2 ms later (500 Hz): frame due, but the 200 Hz poll slot is not
	ASSERT_EQ(fbus.update(now + 2000, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);
	EXPECT_EQ(frame[29], FbusProtocol::FRAME_ID_NULL);

	// A NULL downlink opens no response window: the next frame is not blocked
	ASSERT_EQ(fbus.update(now + 4000, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);
}

TEST(FbusProtocol, ConfigRequestTakesPollSlotAndResponseIsDecoded)
{
	FbusProtocol fbus;
	uint64_t now = 0;
	fbus.reset(now);

	ASSERT_TRUE(fbus.sendConfigWrite(FbusProtocol::XACT_CENTER, 10, 0));
	EXPECT_FALSE(fbus.sendConfigRead(FbusProtocol::XACT_RANGE, 0));	// one pending at a time

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	ASSERT_EQ(fbus.update(now, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);

	// Downlink slot carries the WRITE addressed to physical ID 0x0C
	EXPECT_EQ(frame[28], FbusProtocol::phyIdWithCheckBits(FbusProtocol::XACT_SERVO_PHYS_ID));
	EXPECT_EQ(frame[29], FbusProtocol::FRAME_ID_WRITE);
	EXPECT_EQ(frame[30], 0x00);	// appId 0x6800 little-endian
	EXPECT_EQ(frame[31], 0x68);
	EXPECT_EQ(frame[32], FbusProtocol::XACT_CENTER);	// data = fieldId | value << 8
	EXPECT_EQ(frame[33], 10);
	EXPECT_FALSE(fbus.configRequestPending());

	// Servo RESPONSE: same field/value encoding
	uint8_t response[10];
	buildUplink(response, FbusProtocol::XACT_SERVO_PHYS_ID, FbusProtocol::FRAME_ID_RESPONSE,
		    0x6800, FbusProtocol::XACT_CENTER, 10, 0, 0);
	fbus.processRx(response, sizeof(response), now + 900);

	uint8_t field_id = 0;
	uint32_t value = 0;
	ASSERT_TRUE(fbus.getConfigResponse(field_id, value));
	EXPECT_EQ(field_id, FbusProtocol::XACT_CENTER);
	EXPECT_EQ(value, 10u);

	// One-shot read
	EXPECT_FALSE(fbus.getConfigResponse(field_id, value));
}

TEST(FbusProtocol, CorruptFramesCountCrcErrors)
{
	FbusProtocol fbus;
	uint64_t now = 0;
	fbus.reset(now);

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	ASSERT_EQ(fbus.update(now, frame, sizeof(frame)), FbusProtocol::FRAME_SIZE);

	uint8_t bad[10];
	memcpy(bad, kCapturedUplink, sizeof(bad));
	bad[6] ^= 0xFF;	// flip voltage byte, CRC now wrong
	fbus.processRx(bad, sizeof(bad), now + 800);

	EXPECT_EQ(fbus.crcErrors(), 1u);
	EXPECT_EQ(fbus.framesReceived(), 0u);
	EXPECT_EQ(fbus.sensorCount(), 0);
}

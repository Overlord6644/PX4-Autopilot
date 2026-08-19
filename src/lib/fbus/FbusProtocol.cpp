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

#include "FbusProtocol.hpp"

#include <string.h>
#include <math.h>

namespace
{

// On-wire frame layouts (all little-endian, packed)
struct Control16Frame {
	uint8_t length;			// 0x18
	uint8_t type;			// 0xFF
	uint8_t channel_bytes[22];	// 16 x 11 bits, LSB-first
	uint8_t flags;			// bit0 ch17, bit1 ch18, bit2 sigloss, bit3 failsafe
	uint8_t rssi;
	uint8_t crc;
} __attribute__((__packed__));

struct PollFrame {
	uint8_t length;			// 0x08
	uint8_t phy_id;			// with check bits in bits 5-7
	uint8_t prim;
	uint16_t app_id;
	uint8_t data[4];
	uint8_t crc;
} __attribute__((__packed__));

struct MasterFrame {
	Control16Frame c16;
	PollFrame downlink;
} __attribute__((__packed__));

static_assert(sizeof(Control16Frame) == FbusProtocol::CONTROL_FRAME_SIZE, "control frame size mismatch");
static_assert(sizeof(PollFrame) == FbusProtocol::DOWNLINK_FRAME_SIZE, "poll frame size mismatch");
static_assert(sizeof(MasterFrame) == FbusProtocol::FRAME_SIZE, "master frame size mismatch");

constexpr uint8_t CONTROL16_LENGTH = 0x18;
constexpr uint8_t CONTROL16_TYPE = 0xFF;
constexpr uint8_t DOWNLINK_PAYLOAD_SIZE = 0x08;	// bytes covered by the downlink/uplink CRC
constexpr uint8_t CONTROL_CRC_PAYLOAD_SIZE = 25;	// type..rssi

constexpr uint16_t SBUS_MIN = 192;
constexpr uint16_t SBUS_MAX = 1792;

} // namespace

FbusProtocol::FbusProtocol()
{
	for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
		_channels_us[ch] = CHANNEL_US_NEUTRAL;
	}
}

// rx/frsky_crc.c ported exactly
uint8_t FbusProtocol::frskyCheckSum(const uint8_t *data, uint8_t length)
{
	uint16_t checksum = 0;

	for (uint8_t i = 0; i < length; i++) {
		checksum += data[i];
	}

	while (checksum > 0xFF) {
		checksum = (checksum & 0xFF) + (checksum >> 8);
	}

	return 0xFF - checksum;
}

// smartportMasterPhyIDFillCheckBits
uint8_t FbusProtocol::phyIdWithCheckBits(uint8_t phy_id)
{
	uint8_t b = phy_id & 0x1F;
	b |= (getBit(b, 0) ^ getBit(b, 1) ^ getBit(b, 2)) << 5;
	b |= (getBit(b, 2) ^ getBit(b, 3) ^ getBit(b, 4)) << 6;
	b |= (getBit(b, 0) ^ getBit(b, 2) ^ getBit(b, 4)) << 7;
	return b;
}

int8_t FbusProtocol::stripPhyIdCheckBits(uint8_t phy_id)
{
	const uint8_t raw = phy_id & 0x1F;
	return (phyIdWithCheckBits(raw) == phy_id) ? (int8_t)raw : -1;
}

// fbusMasterConvertToSbus(): us 1000..2000 -> SBUS 192..1792, clamped
uint16_t FbusProtocol::microsecondsToSbus(uint16_t microseconds)
{
	const float scaled = ((float)microseconds - CHANNEL_US_MIN)
			     * (float)(SBUS_MAX - SBUS_MIN)
			     / (float)(CHANNEL_US_MAX - CHANNEL_US_MIN)
			     + (float)SBUS_MIN;
	long value = lroundf(scaled);

	if (value < SBUS_MIN) { value = SBUS_MIN; }

	if (value > SBUS_MAX) { value = SBUS_MAX; }

	return (uint16_t)value;
}

void FbusProtocol::reset(uint64_t now_us)
{
	_phys_ids_found = 0;
	_phys_id_cnt = 0;
	_current_phys_id = 0;
	_discovery_state = DiscoveryState::Scan;
	_next_telemetry_poll_time_us = 0;
	_discovery_end_time_us = now_us + (uint64_t)DISCOVERY_TIME_MS * 1000;
	_next_frame_time_us = now_us;
	_awaiting_response = false;
	_rx_count = 0;
	_echo_state = EchoState::Unknown;
	_echo_len = 0;
	_echo_index = 0;

	for (auto &slot : _servos) {
		slot.has_data = false;
	}
}

void FbusProtocol::setChannel(uint8_t channel, uint16_t microseconds)
{
	if (channel < MAX_CHANNELS) {
		_channels_us[channel] = microseconds;
	}
}

void FbusProtocol::setFrameRate(uint16_t hz)
{
	if (hz < 50) { hz = 50; }

	if (hz > 1000) { hz = 1000; }

	_frame_interval_us = 1000000u / hz;
}

uint8_t FbusProtocol::takeNextScanPhysId()
{
	// Scan 0x00..0x1A; 0x1B (FC common ID) is skipped by the wrap,
	// exactly like fbusMasterTakeNextScanPhysId().
	if (_current_phys_id >= MAX_PHYS_ID) {
		_current_phys_id = 0;
	}

	return _current_phys_id++;
}

size_t FbusProtocol::update(uint64_t now_us, uint8_t *tx_buf, size_t tx_buf_size)
{
	// Response window timeout
	if (_awaiting_response && (int64_t)(now_us - _poll_sent_time_us) > (int64_t)MAX_RESPONSE_DELAY_US) {
		_awaiting_response = false;
		_response_timeouts++;
	}

	// Telemetry staleness (per slot, 1 s)
	for (auto &slot : _servos) {
		if (slot.has_data && (int64_t)(now_us - slot.last_update_us) > (int64_t)SENSOR_STALE_US) {
			slot.has_data = false;
		}
	}

	// Send the next control+downlink frame at the frame rate, but never while
	// still inside the response window of an outstanding poll.
	if ((int64_t)(now_us - _next_frame_time_us) < 0 || _awaiting_response || !_tx_enabled) {
		return 0;
	}

	if (tx_buf_size < FRAME_SIZE) {
		return 0;
	}

	_next_frame_time_us = now_us + _frame_interval_us;
	buildFrame(now_us, tx_buf);

	_tx_bytes += FRAME_SIZE;
	_rx_count = 0;

	// Arm the echo filter with an exact copy of what goes on the wire
	if (_echo_state != EchoState::Absent) {
		memcpy(_echo_frame, tx_buf, FRAME_SIZE);
		_echo_len = FRAME_SIZE;
		_echo_index = 0;
	}

	return FRAME_SIZE;
}

void FbusProtocol::buildFrame(uint64_t now_us, uint8_t *buf)
{
	MasterFrame frame;
	memset(&frame, 0, sizeof(frame));

	// Control frame: pack 16 x 11-bit channels LSB-first
	frame.c16.length = CONTROL16_LENGTH;
	frame.c16.type = CONTROL16_TYPE;

	uint32_t bitpos = 0;

	for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
		const uint16_t sbus = microsecondsToSbus(_channels_us[ch]) & 0x7FF;
		const uint32_t byte_index = bitpos >> 3;
		const uint32_t bit_index = bitpos & 7;
		frame.c16.channel_bytes[byte_index] |= (uint8_t)(sbus << bit_index);
		frame.c16.channel_bytes[byte_index + 1] |= (uint8_t)(sbus >> (8 - bit_index));

		if (bit_index > 5 && byte_index + 2 < sizeof(frame.c16.channel_bytes)) {
			frame.c16.channel_bytes[byte_index + 2] |= (uint8_t)(sbus >> (16 - bit_index));
		}

		bitpos += 11;
	}

	frame.c16.flags = 0;
	frame.c16.rssi = 100;	// fixed, as in fbusMasterPrepareFrame
	frame.c16.crc = frskyCheckSum(&frame.c16.type, CONTROL_CRC_PAYLOAD_SIZE);

	// Downlink slot
	frame.downlink.length = DOWNLINK_PAYLOAD_SIZE;
	bool is_poll = false;

	if ((int64_t)(now_us - _next_telemetry_poll_time_us) < 0) {
		// Between polls: NULL frame
		frame.downlink.phy_id = 0;
		frame.downlink.prim = FRAME_ID_NULL;

	} else if (_config_pending) {
		// Xact config request takes the poll slot (LUA: push(0x0C, prim, appId, data))
		_next_telemetry_poll_time_us = now_us + (1000000u / TELEMETRY_RATE_HZ);
		frame.downlink.phy_id = phyIdWithCheckBits(XACT_SERVO_PHYS_ID);
		frame.downlink.prim = _config_prim;
		frame.downlink.app_id = _config_app_id;
		frame.downlink.data[0] = (uint8_t)(_config_data & 0xFF);
		frame.downlink.data[1] = (uint8_t)((_config_data >> 8) & 0xFF);
		frame.downlink.data[2] = (uint8_t)((_config_data >> 16) & 0xFF);
		frame.downlink.data[3] = (uint8_t)((_config_data >> 24) & 0xFF);
		_config_pending = false;
		is_poll = true;

	} else {
		_next_telemetry_poll_time_us = now_us + (1000000u / TELEMETRY_RATE_HZ);

		if (_discovery_state == DiscoveryState::Scan
		    && (int64_t)(now_us - _discovery_end_time_us) >= 0) {
			_discovery_state = DiscoveryState::Query;
			_phys_id_cnt = 0;
		}

		switch (_discovery_state) {
		case DiscoveryState::Scan:
			frame.downlink.phy_id = phyIdWithCheckBits(takeNextScanPhysId());
			frame.downlink.prim = FRAME_ID_DATA;
			is_poll = true;
			break;

		case DiscoveryState::Query:
			if (_phys_ids_found == 0) {
				// Nothing found: restart discovery
				_discovery_state = DiscoveryState::Scan;
				_current_phys_id = 0;
				_discovery_end_time_us = now_us + (uint64_t)DISCOVERY_TIME_MS * 1000;
				frame.downlink.phy_id = 0;
				frame.downlink.prim = FRAME_ID_NULL;
				break;
			}

			if (_phys_id_cnt >= _phys_ids_found) {
				_phys_id_cnt = 0;
			}

			_current_phys_id = _phys_id_list[_phys_id_cnt];
			frame.downlink.phy_id = phyIdWithCheckBits(_current_phys_id);
			frame.downlink.prim = FRAME_ID_DATA;
			_phys_id_cnt++;
			is_poll = true;
			break;
		}
	}

	frame.downlink.crc = frskyCheckSum(&frame.downlink.phy_id, DOWNLINK_PAYLOAD_SIZE);

	if (is_poll) {
		// Response window: valid 500..3000 us after the poll (rx/fbus.c)
		_awaiting_response = true;
		_poll_sent_time_us = now_us;
	}

	memcpy(buf, &frame, sizeof(frame));
}

void FbusProtocol::processRx(const uint8_t *data, size_t len, uint64_t now_us)
{
	for (size_t i = 0; i < len; i++) {
		handleRxByte(data[i], now_us);
	}
}

void FbusProtocol::handleRxByte(uint8_t byte, uint64_t now_us)
{
	_raw_rx_bytes++;

	// Adaptive TX echo filter: consume bytes that replay the transmitted frame.
	if (_echo_state != EchoState::Absent && _echo_index < _echo_len) {
		if (byte == _echo_frame[_echo_index]) {
			_echo_index++;

			if (_echo_index == _echo_len && _echo_state == EchoState::Unknown) {
				// A full frame echoed back: the port loops TX into RX
				_echo_state = EchoState::Present;
			}

			return;
		}

		if (_echo_state == EchoState::Unknown && _echo_index == 0) {
			// First byte after a transmission is already foreign: no echo on
			// this port (e.g. LPUART single-wire disconnects RX during TX).
			_echo_state = EchoState::Absent;

		} else {
			// Mid-echo mismatch (corrupted echo or a sensor answering early):
			// give up on this frame's echo and treat the byte as data.
			_echo_index = _echo_len;
		}
	}

	_rx_buffer[_rx_count++] = byte;

	if (_rx_count >= RX_BUFFER_SIZE) {
		_rx_count = 0;	// sync error

	} else if (_rx_count >= UPLINK_FRAME_SIZE && _rx_buffer[0] == DOWNLINK_PAYLOAD_SIZE) {
		processUplinkFrame(_rx_buffer, now_us);
		_rx_count = 0;
	}
}

void FbusProtocol::processUplinkFrame(const uint8_t *data, uint64_t now_us)
{
	PollFrame frame;
	memcpy(&frame, data, sizeof(frame));

	const uint8_t chk_sum = frskyCheckSum(&frame.phy_id, DOWNLINK_PAYLOAD_SIZE);

	if (chk_sum != frame.crc) {
		_crc_errors++;
		return;
	}

	_awaiting_response = false;
	_frames_received++;

	const int8_t decoded_phy_id = stripPhyIdCheckBits(frame.phy_id);

	if (decoded_phy_id < 0) {
		return;
	}

	// During scan, any valid response registers the sensor. A freshly detected
	// sensor sends an empty startup frame (prim 0x10, appId 0, data 0) first.
	if (_discovery_state == DiscoveryState::Scan) {
		bool already_in_list = false;

		for (uint8_t i = 0; i < _phys_ids_found; i++) {
			if (_phys_id_list[i] == (uint8_t)decoded_phy_id) {
				already_in_list = true;
				break;
			}
		}

		if (!already_in_list && _phys_ids_found < sizeof(_phys_id_list)) {
			_phys_id_list[_phys_ids_found++] = (uint8_t)decoded_phy_id;
		}
	}

	const uint32_t sensor_data = (uint32_t)frame.data[0]
				     | ((uint32_t)frame.data[1] << 8)
				     | ((uint32_t)frame.data[2] << 16)
				     | ((uint32_t)frame.data[3] << 24);

	// Xact config response: prim 0x32, data = fieldId | (value << 8)
	if (frame.prim == FRAME_ID_RESPONSE
	    && frame.app_id >= SERVO_DATA_BASE
	    && frame.app_id < SERVO_DATA_BASE + MAX_SERVOS) {
		_config_response_field_id = (uint8_t)(sensor_data & 0xFF);
		_config_response_value = sensor_data >> 8;
		_config_response_fresh = true;
		return;
	}

	// Only data frames from real sensors carry telemetry
	if (frame.prim != FRAME_ID_DATA || decoded_phy_id == FC_COMMON_ID) {
		return;
	}

	// Xact servo telemetry, slot keyed by servoId = appId - 0x6800.
	// Unlike the single-servo bench port, ANY physical ID is accepted here:
	// on a shared bus every servo must carry a unique physical ID (FrSky
	// requirement) while the appId identifies the servo.
	if (frame.app_id >= SERVO_DATA_BASE && frame.app_id < SERVO_DATA_BASE + MAX_SERVOS) {
		const uint8_t servo_id = (uint8_t)(frame.app_id - SERVO_DATA_BASE);

		// Ignore the empty startup frame (appId would be 0, filtered above)
		ServoSlot &slot = _servos[servo_id];
		slot.current_deci_a = (uint16_t)(sensor_data & 0xFF);
		slot.voltage_deci_v = (uint16_t)((sensor_data >> 8) & 0xFF);
		slot.temperature_c = (uint8_t)((sensor_data >> 16) & 0xFF);
		slot.last_update_us = now_us;
		slot.has_data = true;
	}
}

bool FbusProtocol::getServoTelemetry(uint8_t servo_id, uint64_t now_us, ServoTelemetry &out) const
{
	if (servo_id >= MAX_SERVOS) {
		return false;
	}

	const ServoSlot &slot = _servos[servo_id];

	if (!slot.has_data || (int64_t)(now_us - slot.last_update_us) > (int64_t)SENSOR_STALE_US) {
		return false;
	}

	out.current_a = slot.current_deci_a / 10.0f;
	out.voltage_v = slot.voltage_deci_v / 10.0f;
	out.temperature_c = slot.temperature_c;
	out.last_update_us = slot.last_update_us;
	return true;
}

bool FbusProtocol::sendConfigRead(uint8_t field_id, uint8_t servo_id)
{
	if (_config_pending || servo_id >= MAX_SERVOS) {
		return false;
	}

	_config_prim = FRAME_ID_READ;
	_config_app_id = SERVO_DATA_BASE + servo_id;
	_config_data = field_id;
	_config_response_fresh = false;
	_config_pending = true;
	return true;
}

bool FbusProtocol::sendConfigWrite(uint8_t field_id, uint32_t value, uint8_t servo_id)
{
	if (_config_pending || servo_id >= MAX_SERVOS) {
		return false;
	}

	_config_prim = FRAME_ID_WRITE;
	_config_app_id = SERVO_DATA_BASE + servo_id;
	_config_data = (uint32_t)field_id | (value << 8);
	_config_response_fresh = false;
	_config_pending = true;
	return true;
}

bool FbusProtocol::sendConfigSave(uint8_t servo_id)
{
	if (_config_pending || servo_id >= MAX_SERVOS) {
		return false;
	}

	// LUA telemetrySave(): WRITE with data = 0x30 (fieldId only, no value)
	_config_prim = FRAME_ID_WRITE;
	_config_app_id = SERVO_DATA_BASE + servo_id;
	_config_data = XACT_SAVE_TO_FLASH;
	_config_response_fresh = false;
	_config_pending = true;
	return true;
}

bool FbusProtocol::getConfigResponse(uint8_t &field_id, uint32_t &value)
{
	if (!_config_response_fresh) {
		return false;
	}

	field_id = _config_response_field_id;
	value = _config_response_value;
	_config_response_fresh = false;
	return true;
}

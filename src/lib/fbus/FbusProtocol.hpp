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
 * @file FbusProtocol.hpp
 *
 * FrSky FBUS (F.Port v2) bus-master protocol core.
 *
 * Platform-free port of the Rotorflight FBUS master (drivers/fbus_master.c,
 * drivers/fbus_sensor.c, rx/fbus.c, rx/frsky_crc.c), by way of the
 * bench-validated Teensy implementation from the MFCA flight-controller
 * firmware (lib/fbus/FBusMaster). The caller owns the serial port and the
 * clock: update() produces the next control+downlink frame to transmit,
 * processRx() consumes every received byte (TX echo included).
 *
 * Bus cycle (timing constants from the Rotorflight source):
 *  - a CONTROL frame (16 x 11-bit channels, SBUS packing) followed
 *    back-to-back by a DOWNLINK (poll) frame, sent at the frame rate
 *  - the downlink slot carries a real poll (prim 0x10 DATA) at the telemetry
 *    rate; otherwise a NULL poll (prim 0x00)
 *  - discovery: scan physical IDs 0x00..0x1A during a 5000 ms window, then
 *    round-robin poll the sensors that answered
 *  - a polled sensor answers with a 10-byte UPLINK frame 500..3000 us after
 *    the poll; later counts as a timeout
 *
 * Xact servo telemetry (appId 0x6800 + servoId):
 *  bits 0-7 current in 0.1 A, bits 8-15 voltage in 0.1 V,
 *  bits 16-23 temperature in 1 degC.
 *
 * TX echo: an STM32 UART in single-wire half-duplex mode (HDSEL) loops the
 * transmitted bytes back into the receiver, while e.g. the i.MX RT LPUART
 * does not. The echo filter is therefore adaptive: incoming bytes that match
 * the transmitted frame byte-for-byte are consumed as echo; the first foreign
 * byte right after a transmission classifies the port as echo-free. See
 * echoState().
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

class FbusProtocol
{
public:
	static constexpr uint32_t BAUDRATE = 460800;

	static constexpr uint8_t MAX_CHANNELS = 16;
	static constexpr uint8_t MAX_SERVOS = 16;		///< servoId 0..15 -> appId 0x6800+n

	static constexpr size_t CONTROL_FRAME_SIZE = 27;	///< [len][type][22 ch bytes][flags][rssi][crc]
	static constexpr size_t DOWNLINK_FRAME_SIZE = 10;	///< [len][phyID][prim][appId lo/hi][d0..d3][crc]
	static constexpr size_t FRAME_SIZE = CONTROL_FRAME_SIZE + DOWNLINK_FRAME_SIZE;
	static constexpr size_t UPLINK_FRAME_SIZE = 10;

	static constexpr uint16_t DEFAULT_FRAME_RATE_HZ = 500;	///< bus limit ~1245 Hz, servo limit 333 Hz
	static constexpr uint16_t TELEMETRY_RATE_HZ = 200;	///< poll slots per second (round-robin)
	static constexpr uint32_t DISCOVERY_TIME_MS = 5000;
	static constexpr uint32_t MAX_RESPONSE_DELAY_US = 3000;
	static constexpr uint64_t SENSOR_STALE_US = 1000000;

	static constexpr uint16_t CHANNEL_US_MIN = 1000;	///< SBUS-representable range
	static constexpr uint16_t CHANNEL_US_MAX = 2000;
	static constexpr uint16_t CHANNEL_US_NEUTRAL = 1500;

	static constexpr uint8_t XACT_SERVO_PHYS_ID = 0x0C;	///< factory default physical ID
	static constexpr uint16_t SERVO_DATA_BASE = 0x6800;

	// Frame IDs (prim)
	enum : uint8_t {
		FRAME_ID_NULL = 0x00,
		FRAME_ID_DATA = 0x10,
		FRAME_ID_READ = 0x30,
		FRAME_ID_WRITE = 0x31,
		FRAME_ID_RESPONSE = 0x32,
	};

	// Xact servo configuration fields (FrSky servo LUA v1.02 dictionary)
	enum XactField : uint8_t {
		XACT_PHYSICAL_ID = 0x00,	///< 0..26
		XACT_SERVO_ID = 0x01,		///< 0..15, selects appId 0x6800+n
		XACT_REFRESH_TIMER = 0x02,	///< telemetry data interval ("Data Rate", ms)
		XACT_RANGE = 0x04,		///< v1.02 fw: 0=120deg 1=90deg 2=180deg (differs on v1.01!)
		XACT_DIRECTION = 0x05,		///< 0=CW 1=CCW
		XACT_PULSE_TYPE = 0x06,		///< 0=1520us/333Hz 1=760us/700Hz
		XACT_CHANNEL_ID = 0x07,		///< 0..23, control-frame channel to follow
		XACT_CENTER = 0x08,		///< -125..125 (signed byte in response)
		XACT_SAVE_TO_FLASH = 0x30,	///< write-only command
	};

	enum class DiscoveryState : uint8_t {
		Scan = 0,	///< scanning physical IDs 0x00..0x1A
		Query,		///< round-robin polling the discovered sensors
	};

	enum class EchoState : uint8_t {
		Unknown = 0,	///< not yet classified (no byte seen after a transmission)
		Present,	///< UART loops TX back into RX (STM32 HDSEL behaviour)
		Absent,		///< receiver is disconnected while transmitting (i.MX RT LPUART)
	};

	struct ServoTelemetry {
		float current_a;
		float voltage_v;
		uint8_t temperature_c;
		uint64_t last_update_us;
	};

	FbusProtocol();

	/// (Re)start discovery and frame scheduling. Call once before the first update().
	void reset(uint64_t now_us);

	/// Channel output in microseconds; clamped to CHANNEL_US_MIN..MAX on the wire.
	void setChannel(uint8_t channel, uint16_t microseconds);

	/// Stop/resume putting frames on the wire. Control and telemetry share the
	/// same frame, so disabling TX also stops telemetry.
	void setTxEnabled(bool enabled) { _tx_enabled = enabled; }
	bool txEnabled() const { return _tx_enabled; }

	/// Frame rate on the wire (clamped 50..1000 Hz). The per-servo update rate
	/// equals the frame rate: the CONTROL frame is a broadcast of all channels.
	void setFrameRate(uint16_t hz);

	/**
	 * Advance protocol time and produce the next frame when due.
	 *
	 * Handles the response-window timeout and telemetry staleness internally.
	 * @return number of bytes written into tx_buf (0 = nothing to transmit now,
	 *         either not due yet or still inside a poll response window).
	 */
	size_t update(uint64_t now_us, uint8_t *tx_buf, size_t tx_buf_size);

	/// Feed received bytes, TX echo included (the adaptive filter removes it).
	void processRx(const uint8_t *data, size_t len, uint64_t now_us);

	/// Latest telemetry for one servoId; false while absent or stale (> 1 s).
	bool getServoTelemetry(uint8_t servo_id, uint64_t now_us, ServoTelemetry &out) const;

	uint8_t sensorCount() const { return _phys_ids_found; }
	uint8_t sensorId(uint8_t index) const { return (index < _phys_ids_found) ? _phys_id_list[index] : 0xFF; }
	DiscoveryState discoveryState() const { return _discovery_state; }
	EchoState echoState() const { return _echo_state; }

	uint32_t responseTimeouts() const { return _response_timeouts; }
	uint32_t crcErrors() const { return _crc_errors; }
	uint32_t rawRxBytes() const { return _raw_rx_bytes; }
	uint32_t txBytes() const { return _tx_bytes; }
	uint32_t framesReceived() const { return _frames_received; }

	/**
	 * Xact servo configuration (FrSky servo LUA / ETHOS dictionary).
	 * Non-blocking: the request takes the next poll slot; retrieve the servo's
	 * RESPONSE with getConfigResponse(). One request pending at a time.
	 * Per the FrSky manual: connect only ONE servo while configuring, finish
	 * with sendConfigSave() to persist. ID changes require re-discovery.
	 * The request is addressed to the factory physical ID 0x0C, matching the
	 * FrSky LUA tool.
	 */
	bool sendConfigRead(uint8_t field_id, uint8_t servo_id = 0);
	bool sendConfigWrite(uint8_t field_id, uint32_t value, uint8_t servo_id = 0);
	bool sendConfigSave(uint8_t servo_id = 0);
	bool configRequestPending() const { return _config_pending; }
	bool getConfigResponse(uint8_t &field_id, uint32_t &value);

	// Protocol helpers, exposed for unit tests
	static uint8_t frskyCheckSum(const uint8_t *data, uint8_t length);
	static uint8_t phyIdWithCheckBits(uint8_t phy_id);
	static int8_t stripPhyIdCheckBits(uint8_t phy_id);	///< -1 if the check bits do not match
	static uint16_t microsecondsToSbus(uint16_t microseconds);

private:
	static constexpr uint8_t MAX_PHYS_ID = 0x1B;	///< 0x1B is the FC common ID, never scanned
	static constexpr uint8_t FC_COMMON_ID = 0x1B;
	static constexpr size_t RX_BUFFER_SIZE = 64;

	static uint8_t getBit(uint8_t value, uint8_t bit) { return (value >> bit) & 1u; }

	uint8_t takeNextScanPhysId();
	void buildFrame(uint64_t now_us, uint8_t *buf);
	void handleRxByte(uint8_t byte, uint64_t now_us);
	void processUplinkFrame(const uint8_t *data, uint64_t now_us);

	// Channel state
	uint16_t _channels_us[MAX_CHANNELS];
	bool _tx_enabled{true};
	uint32_t _frame_interval_us{1000000 / DEFAULT_FRAME_RATE_HZ};

	// Discovery / polling state
	DiscoveryState _discovery_state{DiscoveryState::Scan};
	uint8_t _phys_id_list[MAX_PHYS_ID] {};
	uint8_t _phys_ids_found{0};
	uint8_t _phys_id_cnt{0};
	uint8_t _current_phys_id{0};
	uint64_t _next_telemetry_poll_time_us{0};
	uint64_t _discovery_end_time_us{0};

	// TX scheduling / response window
	uint64_t _next_frame_time_us{0};
	bool _awaiting_response{false};
	uint64_t _poll_sent_time_us{0};

	// RX parsing
	uint8_t _rx_buffer[RX_BUFFER_SIZE] {};
	size_t _rx_count{0};

	// Adaptive TX echo filter
	EchoState _echo_state{EchoState::Unknown};
	uint8_t _echo_frame[FRAME_SIZE] {};
	size_t _echo_len{0};
	size_t _echo_index{0};

	// Counters
	uint32_t _response_timeouts{0};
	uint32_t _crc_errors{0};
	uint32_t _raw_rx_bytes{0};
	uint32_t _tx_bytes{0};
	uint32_t _frames_received{0};

	// Pending Xact configuration request
	bool _config_pending{false};
	uint8_t _config_prim{0};
	uint16_t _config_app_id{0};
	uint32_t _config_data{0};
	bool _config_response_fresh{false};
	uint8_t _config_response_field_id{0};
	uint32_t _config_response_value{0};

	// Per-servo telemetry (indexed by servoId = appId - 0x6800)
	struct ServoSlot {
		uint16_t current_deci_a;
		uint16_t voltage_deci_v;
		uint8_t temperature_c;
		uint64_t last_update_us;
		bool has_data;
	};
	ServoSlot _servos[MAX_SERVOS] {};
};

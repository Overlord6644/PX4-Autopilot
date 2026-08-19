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

#include "FbusServo.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

ModuleBase::Descriptor FbusServo::desc{
	FbusServo::task_spawn,
	FbusServo::custom_command,
	FbusServo::print_usage,
};

FbusServo::FbusServo(const char *device) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")},
	_frame_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": frame interval")}
{
	strncpy(_device, device, sizeof(_device) - 1);
}

FbusServo::~FbusServo()
{
	_serial.close();

	perf_free(_cycle_perf);
	perf_free(_frame_perf);
}

int FbusServo::init()
{
	if (!_serial.setPort(_device)) {
		PX4_ERR("error configuring serial device %s", _device);
		return PX4_ERROR;
	}

	if (!_serial.setBaudrate(FbusProtocol::BAUDRATE)) {
		PX4_ERR("error setting baudrate %lu on %s", (unsigned long)FbusProtocol::BAUDRATE, _device);
		return PX4_ERROR;
	}

	// FBUS is a single-wire half-duplex inverted bus on the port's TX pin.
	// Both calls are unsupported on some platforms (e.g. SITL): warn, don't fail.
	if (!_serial.setSingleWireMode()) {
		PX4_WARN("single-wire mode not supported on %s", _device);
	}

	if (!_serial.setInvertedMode(true)) {
		PX4_WARN("inverted mode not supported on %s", _device);
	}

	if (!_serial.open()) {
		PX4_ERR("error opening serial device %s", _device);
		return PX4_ERROR;
	}

	_fbus.reset(hrt_absolute_time());

	// Self-chaining schedule (ScheduleDelayed at the end of each Run): on the
	// fmu-v6xrt bench, an interval registered from the boot context via
	// ScheduleOnInterval never fired (item attached, period set, 0 runs).
	ScheduleNow();

	PX4_INFO("FBUS master on %s, %lu baud, bus silent until armed/prearmed/actuator test",
		 _device, (unsigned long)FbusProtocol::BAUDRATE);

	return PX4_OK;
}

void FbusServo::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		_serial.close();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	if (!_first_run_done) {
		PX4_INFO("first cycle: enter");
	}

	// Drain the UART (TX echo included; the protocol's adaptive filter eats it).
	// Bounded, and gated on FIONREAD: on the fmu-v6xrt bench a read() on the
	// single-wire port blocked despite O_NONBLOCK and froze the whole work
	// queue - never call read() unless bytes are known to be waiting, and
	// never loop unbounded on a noisy line.
	uint8_t rx_buf[128];

	for (int i = 0; i < 8; i++) {
		if (_serial.bytesAvailable() <= 0) {
			break;
		}

		const ssize_t n = _serial.read(rx_buf, sizeof(rx_buf));

		if (n <= 0) {
			break;
		}

		_fbus.processRx(rx_buf, (size_t)n, hrt_absolute_time());
	}

	if (!_first_run_done) {
		_first_run_done = true;
		PX4_INFO("first cycle: ok");
	}

	_mixing_output.update();	// calls updateOutputs() when functions are assigned

	const uint64_t now = hrt_absolute_time();

	// No CONTROL frame before the first armed/prearmed/actuator-test cycle (or
	// an explicit cfg request): an Xact arms on its first frame and never
	// returns to limp, so the bus stays silent on the pad and across reboots.
	if (!_bus_active) {
		const actuator_armed_s &armed = _mixing_output.armed();

		if (armed.armed || armed.prearmed || _mixing_output.isActuatorTestRunning()) {
			_bus_active = true;
			_fbus.reset(now);
			PX4_INFO("bus activated");
		}
	}

	processConfigRequest(now);	// may also activate the bus

	// The wire pump lives here, not in updateOutputs(): MixingOutput does not
	// call updateOutputs() while no output function is assigned, and the bus
	// must still run for servo provisioning on a freshly configured board.
	if (_bus_active) {
		uint8_t frame[FbusProtocol::FRAME_SIZE];
		const size_t len = _fbus.update(now, frame, sizeof(frame));

		if (len > 0 && _serial.write(frame, len) == (ssize_t)len) {
			perf_count(_frame_perf);
		}
	}

	if (_bus_active && (now - _last_status_pub) >= STATUS_PUB_INTERVAL_US) {
		publishServoStatus(now);
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
	}

	_mixing_output.updateSubscriptions(false);

	perf_end(_cycle_perf);

	// Re-arm the loop (self-chaining keeps the cadence independent of any
	// uORB callback and of the periodic hrt registration path)
	ScheduleDelayed(SCHEDULE_INTERVAL_US);
}

void FbusServo::processConfigRequest(uint64_t now)
{
	const CfgState state = (CfgState)_cfg_state.load();

	if (state == CfgState::Queued) {
		if (_mixing_output.armed().armed) {
			_cfg_state.store((int)CfgState::RejectedArmed);
			return;
		}

		if (!_bus_active) {
			// Deliberate operator action on the bench: wake the bus. Any servo
			// attached from here on is armed by the control frames.
			_bus_active = true;
			_fbus.reset(now);
			PX4_WARN("bus activated for servo configuration");
		}

		bool ok = false;

		switch (_cfg_op) {
		case CfgOp::Read:
			ok = _fbus.sendConfigRead(_cfg_field, _cfg_servo_id);
			break;

		case CfgOp::Write:
			ok = _fbus.sendConfigWrite(_cfg_field, _cfg_value, _cfg_servo_id);
			break;

		case CfgOp::Save:
			ok = _fbus.sendConfigSave(_cfg_servo_id);
			break;
		}

		if (!ok) {
			_cfg_state.store((int)CfgState::Failed);
			return;
		}

		_cfg_sent_time = now;
		_cfg_resp_received = false;
		_cfg_state.store((int)CfgState::InFlight);

	} else if (state == CfgState::InFlight) {
		uint8_t field = 0;
		uint32_t value = 0;

		if (_fbus.getConfigResponse(field, value)) {
			_cfg_resp_field = field;
			_cfg_resp_value = value;
			_cfg_resp_received = true;
			_cfg_state.store((int)CfgState::Done);

		} else if ((now - _cfg_sent_time) > 2000000) {
			// Save is a write-only command: not every servo firmware answers it,
			// so a sent save without response still counts as done.
			_cfg_state.store((int)(_cfg_op == CfgOp::Save ? CfgState::Done : CfgState::Timeout));
		}
	}
}

int FbusServo::runConfigRequest(CfgOp op, uint8_t field, uint32_t value, uint8_t servo_id)
{
	if (_cfg_state.load() != (int)CfgState::Idle) {
		PX4_ERR("configuration request already pending");
		return PX4_ERROR;
	}

	_cfg_op = op;
	_cfg_field = field;
	_cfg_value = value;
	_cfg_servo_id = servo_id;
	_cfg_state.store((int)CfgState::Queued);

	// Poll from the shell thread until the work queue reaches a terminal state
	CfgState state = CfgState::Queued;

	for (int i = 0; i < 60; i++) {
		px4_usleep(50000);
		state = (CfgState)_cfg_state.load();

		if (state != CfgState::Queued && state != CfgState::InFlight) {
			break;
		}
	}

	int ret = PX4_ERROR;

	switch (state) {
	case CfgState::Done:
		if (_cfg_resp_received) {
			PX4_INFO("field 0x%02X = %lu", _cfg_resp_field, (unsigned long)_cfg_resp_value);

			if (_cfg_resp_field == FbusProtocol::XACT_CENTER && _cfg_resp_value > 125) {
				PX4_INFO("center (signed): %ld", (long)_cfg_resp_value - 256);
			}

		} else {
			PX4_INFO("save command sent (no response expected)");
		}

		ret = PX4_OK;
		break;

	case CfgState::RejectedArmed:
		PX4_ERR("rejected: vehicle is armed");
		break;

	case CfgState::Timeout:
		PX4_ERR("no response from servo (one servo on the bus? correct servo id?)");
		break;

	case CfgState::Failed:
		PX4_ERR("request refused by the protocol core");
		break;

	default:
		PX4_ERR("driver did not process the request (module running?)");
		break;
	}

	_cfg_state.store((int)CfgState::Idle);
	return ret;
}

void FbusServo::publishServoStatus(uint64_t now)
{
	servo_status_s status{};

	status.counter = _status_counter++;
	status.servo_count = FBUS_OUTPUT_CHANNELS;
	status.connection_type = servo_status_s::CONNECTION_TYPE_FBUS;

	// By provisioning convention the Xact servoId equals the FBUS channel it
	// follows, so telemetry slot i reports on output channel i.
	for (uint8_t i = 0; i < FBUS_OUTPUT_CHANNELS; i++) {
		servo_report_s &report = status.servo[i];
		report.actuator_function = (uint8_t)_mixing_output.outputFunction(i);

		FbusProtocol::ServoTelemetry telem{};

		if (_fbus.getServoTelemetry(i, now, telem)) {
			report.voltage_v = telem.voltage_v;
			report.current_a = telem.current_a;
			report.temperature_degc = telem.temperature_c;
			report.telemetry_online = true;
			report.timestamp = telem.last_update_us;
			status.servo_online_flags |= (uint16_t)(1u << i);

		} else {
			report.temperature_degc = INT16_MIN;
			report.telemetry_online = false;
			report.timestamp = now;
		}
	}

	status.timestamp = hrt_absolute_time();
	_servo_status_pub.publish(status);
	_last_status_pub = now;
}

bool FbusServo::updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs,
			      unsigned num_control_groups_updated)
{
	// Only latch the channel values; the wire pump runs from Run() so the bus
	// also works while no output function is assigned (servo provisioning).
	for (unsigned i = 0; i < num_outputs && i < FBUS_OUTPUT_CHANNELS; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			_fbus.setChannel(i, (uint16_t)lroundf(outputs[i]));

		} else {
			_fbus.setChannel(i, FbusProtocol::CHANNEL_US_NEUTRAL);
		}
	}

	return true;
}

int FbusServo::print_status()
{
	int ret = ModuleBase::print_status();

	PX4_INFO("device: %s @ %lu baud", _device, (unsigned long)FbusProtocol::BAUDRATE);
	PX4_INFO("bus active: %s", _bus_active ? "yes (transmitting)" : "no (silent until armed/prearmed/test)");

	const char *echo = "unknown";

	switch (_fbus.echoState()) {
	case FbusProtocol::EchoState::Present:
		echo = "present (filtered)";
		break;

	case FbusProtocol::EchoState::Absent:
		echo = "absent";
		break;

	default:
		break;
	}

	PX4_INFO("tx: %lu bytes, rx: %lu bytes, echo: %s",
		 (unsigned long)_fbus.txBytes(), (unsigned long)_fbus.rawRxBytes(), echo);
	PX4_INFO("uplink frames: %lu, crc errors: %lu, poll timeouts: %lu",
		 (unsigned long)_fbus.framesReceived(), (unsigned long)_fbus.crcErrors(),
		 (unsigned long)_fbus.responseTimeouts());
	PX4_INFO("discovery: %s, sensors found: %u",
		 _fbus.discoveryState() == FbusProtocol::DiscoveryState::Scan ? "scanning" : "polling",
		 _fbus.sensorCount());

	for (uint8_t i = 0; i < _fbus.sensorCount(); i++) {
		PX4_INFO("  sensor %u: physical ID 0x%02X", i, _fbus.sensorId(i));
	}

	const uint64_t now = hrt_absolute_time();

	for (uint8_t servo_id = 0; servo_id < FbusProtocol::MAX_SERVOS; servo_id++) {
		FbusProtocol::ServoTelemetry telem{};

		if (_fbus.getServoTelemetry(servo_id, now, telem)) {
			PX4_INFO("  servo %u: %.1f V, %.1f A, %u degC", servo_id,
				 (double)telem.voltage_v, (double)telem.current_a, telem.temperature_c);
		}
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_frame_perf);

	_mixing_output.printStatus();

	return ret;
}

namespace
{

// Xact configuration fields addressable by name from the CLI
const struct {
	const char *name;
	uint8_t id;
} kCfgFields[] = {
	{"physid", FbusProtocol::XACT_PHYSICAL_ID},
	{"servoid", FbusProtocol::XACT_SERVO_ID},
	{"rate", FbusProtocol::XACT_REFRESH_TIMER},
	{"range", FbusProtocol::XACT_RANGE},
	{"dir", FbusProtocol::XACT_DIRECTION},
	{"pulse", FbusProtocol::XACT_PULSE_TYPE},
	{"channel", FbusProtocol::XACT_CHANNEL_ID},
	{"center", FbusProtocol::XACT_CENTER},
};

bool parseCfgField(const char *arg, uint8_t &field)
{
	for (const auto &f : kCfgFields) {
		if (strcmp(arg, f.name) == 0) {
			field = f.id;
			return true;
		}
	}

	char *end = nullptr;
	const unsigned long value = strtoul(arg, &end, 0);

	if (end != nullptr && *end == '\0' && value <= 0xFF) {
		field = (uint8_t)value;
		return true;
	}

	return false;
}

} // namespace

int FbusServo::custom_command(int argc, char **argv)
{
	if (argc >= 1 && strcmp(argv[0], "cfg") == 0) {
		auto *instance = static_cast<FbusServo *>(desc.object.load());

		if (!is_running(desc) || instance == nullptr) {
			PX4_ERR("not running");
			return PX4_ERROR;
		}

		if (argc >= 3 && strcmp(argv[1], "read") == 0) {
			uint8_t field = 0;

			if (!parseCfgField(argv[2], field)) {
				return print_usage("unknown field");
			}

			const uint8_t servo_id = (argc >= 4) ? (uint8_t)strtoul(argv[3], nullptr, 0) : 0;
			return instance->runConfigRequest(CfgOp::Read, field, 0, servo_id);
		}

		if (argc >= 4 && strcmp(argv[1], "write") == 0) {
			uint8_t field = 0;

			if (!parseCfgField(argv[2], field)) {
				return print_usage("unknown field");
			}

			// Center is a signed byte (-125..125): encode negatives on 8 bits
			const long value = strtol(argv[3], nullptr, 0);
			const uint32_t encoded = (value < 0) ? (uint32_t)(value & 0xFF) : (uint32_t)value;

			const uint8_t servo_id = (argc >= 5) ? (uint8_t)strtoul(argv[4], nullptr, 0) : 0;
			return instance->runConfigRequest(CfgOp::Write, field, encoded, servo_id);
		}

		if (argc >= 2 && strcmp(argv[1], "save") == 0) {
			const uint8_t servo_id = (argc >= 3) ? (uint8_t)strtoul(argv[2], nullptr, 0) : 0;
			return instance->runConfigRequest(CfgOp::Save, 0, 0, servo_id);
		}

		return print_usage("usage: fbus cfg read|write|save ...");
	}

	return print_usage("unknown command");
}

int FbusServo::task_spawn(int argc, char **argv)
{
	const char *device = nullptr;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			print_usage("unrecognized option");
			return PX4_ERROR;
		}
	}

	if (device == nullptr || device[0] == '\0') {
		print_usage("missing device");
		return PX4_ERROR;
	}

	auto *instance = new FbusServo(device);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int FbusServo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Actuator output driver for FrSky FBUS (F.Port v2) bus servos (Xact W56xx).

Drives up to 8 servos over one single-wire half-duplex inverted UART at
460800 baud. Servos are assigned with the FBUS_SV_FUNCx output functions in
the standard actuator configuration. The bus stays silent until the vehicle
is armed or prearmed or an actuator test runs (FBUS has no disarm: a servo
arms on its first frame and never returns to limp - a hard disarm requires
cutting the servo power rail).

Wire the bus to the TX pin of the selected serial port (single-wire
half-duplex lives on TX); leave RX unconnected. Select the port with the
FBUS_CFG parameter.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fbus", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("cfg", "Xact servo configuration (disarmed only, ONE servo on the bus)");
	PRINT_MODULE_USAGE_ARG("read <field> [servo_id]", "Read a field (physid|servoid|rate|range|dir|pulse|channel|center or numeric id)", true);
	PRINT_MODULE_USAGE_ARG("write <field> <value> [servo_id]", "Write a field (verify enum order with a read first: Range differs between servo firmwares)", true);
	PRINT_MODULE_USAGE_ARG("save [servo_id]", "Persist the configuration to the servo flash", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fbus_main(int argc, char *argv[])
{
	return ModuleBase::main(FbusServo::desc, argc, argv);
}

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

#include <math.h>
#include <string.h>

ModuleBase::Descriptor FbusServo::desc{
	FbusServo::task_spawn,
	FbusServo::custom_command,
	FbusServo::print_usage,
};

FbusServo::FbusServo(const char *device) :
	OutputModuleInterface(MODULE_NAME, px4::serial_port_to_wq(device)),
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

	ScheduleOnInterval(SCHEDULE_INTERVAL_US);

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

	// Drain the UART (TX echo included; the protocol's adaptive filter eats it)
	uint8_t rx_buf[128];
	ssize_t n;

	while ((n = _serial.read(rx_buf, sizeof(rx_buf))) > 0) {
		_fbus.processRx(rx_buf, (size_t)n, hrt_absolute_time());
	}

	_mixing_output.update();	// calls updateOutputs()

	const uint64_t now = hrt_absolute_time();

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
	const uint64_t now = hrt_absolute_time();

	// No CONTROL frame before the first armed/prearmed/actuator-test cycle:
	// an Xact arms on its first frame and never returns to limp, so the bus
	// stays silent on the pad and across FC reboots.
	if (!_bus_active) {
		const actuator_armed_s &armed = _mixing_output.armed();

		if (armed.armed || armed.prearmed || _mixing_output.isActuatorTestRunning()) {
			_bus_active = true;
			_fbus.reset(now);
			PX4_INFO("bus activated");

		} else {
			return false;
		}
	}

	for (unsigned i = 0; i < num_outputs && i < FBUS_OUTPUT_CHANNELS; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			_fbus.setChannel(i, (uint16_t)lroundf(outputs[i]));

		} else {
			_fbus.setChannel(i, FbusProtocol::CHANNEL_US_NEUTRAL);
		}
	}

	uint8_t frame[FbusProtocol::FRAME_SIZE];
	const size_t len = _fbus.update(now, frame, sizeof(frame));

	if (len > 0) {
		if (_serial.write(frame, len) == (ssize_t)len) {
			perf_count(_frame_perf);
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

int FbusServo::custom_command(int argc, char **argv)
{
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
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fbus_main(int argc, char *argv[])
{
	return ModuleBase::main(FbusServo::desc, argc, argv);
}

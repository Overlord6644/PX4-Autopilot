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
 * @file FbusServo.hpp
 *
 * Actuator output driver for FrSky FBUS (F.Port v2) bus servos (Xact W56xx).
 *
 * Drives up to 8 servos over one single-wire half-duplex inverted UART at
 * 460800 baud, as a MixingOutput consumer (param prefix FBUS_SV): servos are
 * assigned output functions in the standard actuator configuration, like the
 * DroneCAN servo outputs. The wire runs at a deterministic frame rate
 * (SchedulingPolicy::Disabled + own interval), independent of the
 * rate-control loop.
 *
 * Safety semantics (see docs/fbus_plan.md): FBUS has no disarm - a servo
 * arms on its first CONTROL frame and never returns to limp. The bus
 * therefore stays SILENT until the vehicle is armed or prearmed or an
 * actuator test runs, so servos on the pad stay limp and an FC reboot cannot
 * re-arm them. Once started, the bus keeps transmitting (disarmed values
 * from FBUS_SV_DISx). A hard disarm after arming requires cutting the servo
 * power rail.
 */

#pragma once

#include <lib/fbus/FbusProtocol.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

class FbusServo final : public ModuleBase, public OutputModuleInterface
{
public:
	static constexpr uint8_t FBUS_OUTPUT_CHANNELS = 8;

	static ModuleBase::Descriptor desc;

	explicit FbusServo(const char *device);
	~FbusServo() override;

	static int task_spawn(int argc, char **argv);
	static int custom_command(int argc, char **argv);
	static int print_usage(const char *reason = nullptr);

	int init();
	int print_status() override;

	bool updateOutputs(float outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

private:
	// Run twice per bus frame so the protocol paces the wire, not the scheduler
	static constexpr uint32_t SCHEDULE_INTERVAL_US = 1000000 / (2 * FbusProtocol::DEFAULT_FRAME_RATE_HZ);

	void Run() override;

	device::Serial _serial{};
	char _device[32] {};

	FbusProtocol _fbus{};
	MixingOutput _mixing_output{"FBUS_SV", FBUS_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Disabled, false, false};

	bool _bus_active{false};	///< latched on first armed/prearmed/actuator-test cycle

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	perf_counter_t _cycle_perf;
	perf_counter_t _frame_perf;
};

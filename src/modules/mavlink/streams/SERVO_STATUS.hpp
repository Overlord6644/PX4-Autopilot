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

#ifndef SERVO_STATUS_HPP
#define SERVO_STATUS_HPP

// SERVO_STATUS is a custom message (id 52000) carried by the mavlink
// submodule fork (Overlord6644/mavlink, fbus-servo-status branch).
#if defined(MAVLINK_MSG_ID_SERVO_STATUS)

#include <uORB/Subscription.hpp>
#include <uORB/topics/servo_status.h>

#include <math.h>

class MavlinkStreamServoStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamServoStatus(mavlink); }

	static constexpr const char *get_name_static() { return "SERVO_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_SERVO_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _servo_status_sub.advertised() ? MAVLINK_MSG_ID_SERVO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamServoStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _servo_status_sub{ORB_ID(servo_status)};

	static constexpr uint8_t SERVOS_PER_MSG = 8;	// MAVLINK_MSG_SERVO_STATUS_FIELD_VOLTAGE_LEN

	bool send() override
	{
		servo_status_s status;

		if (_servo_status_sub.update(&status)) {
			mavlink_servo_status_t msg{};

			msg.time_usec = status.timestamp;
			msg.count = (status.servo_count < SERVOS_PER_MSG) ? status.servo_count : SERVOS_PER_MSG;
			msg.online_flags = status.servo_online_flags;

			for (uint8_t i = 0; i < msg.count && i < servo_status_s::CONNECTED_SERVO_MAX; i++) {
				msg.servo_function[i] = status.servo[i].actuator_function;

				if (status.servo[i].telemetry_online) {
					msg.voltage[i] = status.servo[i].voltage_v;
					msg.current[i] = status.servo[i].current_a;
					msg.temperature[i] = status.servo[i].temperature_degc;

				} else {
					msg.voltage[i] = (float)NAN;
					msg.current[i] = (float)NAN;
					msg.temperature[i] = INT16_MIN;
				}
			}

			mavlink_msg_servo_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // MAVLINK_MSG_ID_SERVO_STATUS
#endif // SERVO_STATUS_HPP

/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

/* uorb_to_msp.hpp
 *
 * Declaration of functions which translate UORB messages into MSP specific structures.
 */

#pragma once

// basic types
#include <cmath>

// UORB topic structs
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/esc_status.h>

// PX4 events interface
#include <px4_platform_common/events.h>

// MSP structs
#include "msp_defines.h"
#include "MessageDisplay/MessageDisplay.hpp"

namespace msp_osd
{

// construct an MSP_FC_VARIANT struct
msp_fc_variant_t construct_FC_VARIANT();

// construct an MSP_STATUS struct
msp_status_BF_t construct_STATUS(const vehicle_status_s &vehicle_status);

// construct an MSP_ANALOG struct
msp_analog_t construct_ANALOG(const battery_status_s &battery_status, const input_rc_s &input_rc);

msp_rendor_rssi_t construct_rendor_RSSI(const rc_channels_s &rc_channels);

// construct an MSP_BATTERY_STATE struct
msp_battery_state_t construct_BATTERY_STATE(const battery_status_s &battery_status);

msp_rendor_battery_state_t construct_rendor_BATTERY_STATE(const battery_status_s &battery_status);

// construct an MSP_RAW_GPS struct
msp_raw_gps_t construct_RAW_GPS(const sensor_gps_s &vehicle_gps_position,
				const airspeed_validated_s &airspeed_validated);

// construct an MSP_COMP_GPS struct
msp_comp_gps_t construct_COMP_GPS(const home_position_s &home_position,
				  const vehicle_global_position_s &vehicle_global_position,
				  const bool heartbeat);

msp_rendor_latitude_t construct_rendor_GPS_LAT(const sensor_gps_s &vehicle_gps_position);

msp_rendor_longitude_t construct_rendor_GPS_LON(const sensor_gps_s &vehicle_gps_position);

msp_rendor_satellites_used_t construct_rendor_GPS_NUM(const sensor_gps_s &vehicle_gps_position);

// construct an MSP_ATTITUDE struct
msp_attitude_t construct_ATTITUDE(const vehicle_attitude_s &vehicle_attitude);

msp_rendor_pitch_t  construct_rendor_PITCH(const vehicle_attitude_s &vehicle_attitude);

msp_rendor_roll_t  construct_rendor_ROLL(const vehicle_attitude_s &vehicle_attitude);

void construct_rendor_HORIZON_SIDEBARS(const vehicle_attitude_s &vehicle_attitude,
	msp_rendor_horizon_sidebar_t &left, msp_rendor_horizon_sidebar_t &right);

// construct an MSP_ALTITUDE struct
msp_altitude_t construct_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
				  const vehicle_local_position_s &vehicle_local_position);

msp_rendor_altitude_t construct_Rendor_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
		const vehicle_local_position_s &vehicle_local_position);

msp_rendor_distanceToHome_t construct_rendor_distanceToHome(const home_position_s &home_position,
		const vehicle_global_position_s &vehicle_global_position);

// construct an MSP_ESC_SENSOR_DATA struct
msp_esc_sensor_data_dji_t construct_ESC_SENSOR_DATA();

// construct an MSP_RC struct
msp_rc_t construct_MSP_RC(const input_rc_s &input_rc);

// construct an MSP_STATUS struct
msp_status_t construct_MSP_STATUS(const vehicle_status_s &vehicle_status);

// Additional rendor functions for missing OSD elements
msp_rendor_gps_speed_t construct_rendor_GPS_SPEED(const sensor_gps_s &vehicle_gps_position, int32_t speed_unit, int32_t speed_type);

msp_rendor_airspeed_t construct_rendor_AIRSPEED(const airspeed_validated_s &airspeed_validated, int32_t speed_unit, int32_t airspeed_source);

msp_rendor_home_direction_t construct_rendor_HOME_DIR(const home_position_s &home_position,
		const vehicle_global_position_s &vehicle_global_position,
		const vehicle_attitude_s &vehicle_attitude);

msp_rendor_power_t construct_rendor_POWER(const battery_status_s &battery_status);

msp_rendor_vspeed_t construct_rendor_VSPEED(const vehicle_local_position_s &vehicle_local_position);

msp_rendor_flight_mode_t construct_rendor_FLIGHT_MODE(const vehicle_status_s &vehicle_status, bool use_short_format);

msp_rendor_crosshairs_t construct_rendor_CROSSHAIRS();

void construct_rendor_ARTIFICIAL_HORIZON(const vehicle_attitude_s &vehicle_attitude, int crosshair_y, int crosshair_x, int width,
	float pitch_offset, msp_rendor_horizon_bar_t &horizon_bar, msp_rendor_horizon_cursor_t &cursor_left, msp_rendor_horizon_cursor_t &cursor_right);

// Construct pitch ladder with fixed horizon and moving graduations
// Returns number of lines generated (for iteration)
// ladder_side: 1=left, 2=right, 3=both
// ladder_step: angular spacing between graduations (5, 10, 15, 20, 30 degrees)
// pitch_offset: offset in degrees to compensate for PX4 mounting misalignment
int construct_rendor_PITCH_LADDER(const vehicle_attitude_s &vehicle_attitude,
	int crosshair_y, int crosshair_x, int ladder_width, int ladder_side, int ladder_step,
	float pitch_offset, msp_rendor_pitch_ladder_line_t lines[], int max_lines);

void construct_rendor_HEADING(const vehicle_attitude_s &vehicle_attitude, const home_position_s &home_position, const vehicle_global_position_s &vehicle_global_position, msp_rendor_heading_t &row1, msp_rendor_heading_t &row2);

msp_rendor_arming_t construct_rendor_ARMING(const vehicle_status_s &vehicle_status);

msp_rendor_warning_t construct_rendor_WARNING(const log_message_s &log_message, const int log_level);

msp_rendor_esc_temp_t construct_rendor_ESC_TEMP(const esc_status_s &esc_status);

msp_rendor_esc_rpm_t construct_rendor_ESC_RPM(const esc_status_s &esc_status);

msp_rendor_esc_amp_t construct_rendor_ESC_AMP(const esc_status_s &esc_status);

msp_rendor_current_t construct_rendor_CURRENT(const battery_status_s &battery_status);

msp_rendor_mah_t construct_rendor_MAH(const battery_status_s &battery_status);

msp_rendor_stall_warning_t construct_rendor_STALL_WARNING(const airspeed_validated_s &airspeed_validated, float fw_airspd_min, bool test_mode);

msp_rendor_alt_warning_t construct_rendor_ALT_MAX_WARNING(float current_altitude, float max_altitude, bool force_test = false);

msp_rendor_g_meter_t construct_rendor_G_METER(const sensor_combined_s &sensor_combined, float max_g);

msp_rendor_pullpot_t construct_rendor_PULL_POT(const airspeed_validated_s &airspeed_validated,
		const sensor_combined_s &sensor_combined, int32_t source, float vs_m_s, float csv_gmax);

msp_rendor_throttle_t construct_rendor_THROTTLE(const input_rc_s &input_rc);

} // namespace msp_osd

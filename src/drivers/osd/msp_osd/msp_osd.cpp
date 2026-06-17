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

/* Notes:
 *  - Currently there's a lot of wasted processing here if certain displays are enabled.
 *    A relatively low-hanging fruit would be figuring out which display elements require
 *    information from what UORB topics and disable if the information isn't displayed.
 * 	(this is complicated by the fact that it's not a one-to-one mapping...)
 */

#include "msp_osd.hpp"

#include "msp_defines.h"

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <ctype.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>

#include <lib/geo/geo.h>

#include "MspV1.hpp"

//OSD elements positions are now configured via parameters
// Position encoding: 2048 + X + (Y * 32)
// X range: 0-29, Y range: 0-15
// Position 0 = hidden

#define OSD_GRID_COL_MAX (59) // From betaflight-configurator OSD tab
#define OSD_GRID_ROW_MAX (21) // From betaflight-configurator OSD tab

// Helper template to calculate dynamic struct size based on str_length
template<typename T>
static inline size_t calc_rendor_size(const T& rendor) {
	return offsetof(T, str) + rendor.str_length;
}

typedef enum {
	MSP_DP_HEARTBEAT = 0,         // Release the display after clearing and updating
	MSP_DP_RELEASE = 1,         // Release the display after clearing and updating
	MSP_DP_CLEAR_SCREEN = 2,    // Clear the display
	MSP_DP_WRITE_STRING = 3,    // Write a string at given coordinates
	MSP_DP_DRAW_SCREEN = 4,     // Trigger a screen draw
	MSP_DP_OPTIONS = 5,         // Not used by Betaflight. Reserved by Ardupilot and INAV
	MSP_DP_SYS = 6,             // Display system element displayportSystemElement_e at given coordinates
	MSP_DP_COUNT,
} displayportMspSubCommand;


#define OSD_GRID_COL_MAX (59) // From betaflight-configurator OSD tab
#define OSD_GRID_ROW_MAX (21) // From betaflight-configurator OSD tab


MspOsd::MspOsd(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	// back up device name for connection later
	strcpy(_device, device);

	// Configure warning display scrolling
	_warning_display.set_period(125'000);  // 125ms scroll period
	_warning_display.set_dwell(500'000);   // 500ms dwell at start

	// _is_initialized = true;
	PX4_INFO("MSP OSD running on %s", _device);
}

MspOsd::~MspOsd()
{
}

bool MspOsd::init()
{
	updateParams();
	parameters_update();

	ScheduleOnInterval(100_ms);

	return true;
}

void MspOsd::SendConfig()
{
	msp_osd_config_t msp_osd_config;

	msp_osd_config.units = 0;
	msp_osd_config.osd_item_count = 56;
	msp_osd_config.osd_stat_count = 24;
	msp_osd_config.osd_timer_count = 2;
	msp_osd_config.osd_warning_count = 16;              // 16
	msp_osd_config.osd_profile_count = 1;              // 1
	msp_osd_config.osdprofileindex = 1;                // 1
	msp_osd_config.overlay_radio_mode = 0;             //  0

	// display conditional elements - use parameters for positions
	msp_osd_config.osd_craft_name_pos = enabled(SymbolIndex::CRAFT_NAME) ?
		calculate_position(_param_osd_msg_x.get(), _param_osd_msg_y.get()) : 0;
	msp_osd_config.osd_disarmed_pos = enabled(SymbolIndex::DISARMED) ?
		calculate_position(_param_osd_disarm_x.get(), _param_osd_disarm_y.get()) : 0;
	msp_osd_config.osd_gps_lat_pos = enabled(SymbolIndex::GPS_LAT) ?
		calculate_position(_param_osd_gpslat_x.get(), _param_osd_gpslat_y.get()) : 0;
	msp_osd_config.osd_gps_lon_pos = enabled(SymbolIndex::GPS_LON) ?
		calculate_position(_param_osd_gpslon_x.get(), _param_osd_gpslon_y.get()) : 0;
	msp_osd_config.osd_gps_sats_pos = enabled(SymbolIndex::GPS_SATS) ?
		calculate_position(_param_osd_sats_x.get(), _param_osd_sats_y.get()) : 0;
	msp_osd_config.osd_gps_speed_pos = enabled(SymbolIndex::GPS_SPEED) ?
		calculate_position(_param_osd_gspd_x.get(), _param_osd_gspd_y.get()) : 0;
	msp_osd_config.osd_home_dist_pos = enabled(SymbolIndex::HOME_DIST) ?
		calculate_position(_param_osd_hdist_x.get(), _param_osd_hdist_y.get()) : 0;
	msp_osd_config.osd_home_dir_pos = enabled(SymbolIndex::HOME_DIR) ?
		calculate_position(_param_osd_hdir_x.get(), _param_osd_hdir_y.get()) : 0;
	msp_osd_config.osd_main_batt_voltage_pos = enabled(SymbolIndex::MAIN_BATT_VOLTAGE) ?
		calculate_position(_param_osd_batvolt_x.get(), _param_osd_batvolt_y.get()) : 0;
	msp_osd_config.osd_current_draw_pos = enabled(SymbolIndex::CURRENT_DRAW) ?
		calculate_position(_param_osd_current_x.get(), _param_osd_current_y.get()) : 0;
	msp_osd_config.osd_mah_drawn_pos = enabled(SymbolIndex::MAH_DRAWN) ?
		calculate_position(_param_osd_mah_x.get(), _param_osd_mah_y.get()) : 0;
	msp_osd_config.osd_rssi_value_pos = enabled(SymbolIndex::RSSI_VALUE) ?
		calculate_position(_param_osd_rssi_x.get(), _param_osd_rssi_y.get()) : 0;
	msp_osd_config.osd_altitude_pos = enabled(SymbolIndex::ALTITUDE) ?
		calculate_position(_param_osd_alt_x.get(), _param_osd_alt_y.get()) : 0;
	msp_osd_config.osd_numerical_vario_pos = enabled(SymbolIndex::NUMERICAL_VARIO) ?
		calculate_position(_param_osd_vario_x.get(), _param_osd_vario_y.get()) : 0;

	msp_osd_config.osd_power_pos = enabled(SymbolIndex::POWER) ?
		calculate_position(_param_osd_power_x.get(), _param_osd_power_y.get()) : 0;
	msp_osd_config.osd_throttle_pos_pos = enabled(SymbolIndex::THROTTLE) ?
		calculate_position(_param_osd_thr_x.get(), _param_osd_thr_y.get()) : 0;

	// crosshairs position (fixed center: 26, 10)
	msp_osd_config.osd_crosshairs_pos = enabled(SymbolIndex::CROSSHAIRS) ?
		calculate_position(26, 10 - _param_osd_ch_height.get()) : 0;

	// possibly available, but not currently used
	msp_osd_config.osd_flymode_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_tmp_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_horizon_sidebars_pos = 		LOCATION_HIDDEN;

	// Not implemented or not available
	msp_osd_config.osd_artificial_horizon_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_1_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_2_pos = 			LOCATION_HIDDEN;
	// throttle position is configured above when THROTTLE symbol is enabled
	msp_osd_config.osd_vtx_channel_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_yaw_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pidrate_profile_pos =		LOCATION_HIDDEN;
	msp_osd_config.osd_warnings_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_debug_pos = 				LOCATION_HIDDEN;
	msp_osd_config.osd_main_batt_usage_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_numerical_heading_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_compass_bar_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_remaining_time_estimate_pos = 	LOCATION_HIDDEN;
	msp_osd_config.osd_rtc_datetime_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_adjustment_range_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_core_temperature_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_anti_gravity_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_g_force_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_motor_diag_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_log_status_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flip_arrow_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_link_quality_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flight_dist_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_left_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_right_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_display_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_freq_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rate_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_pid_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_profile_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rssi_dbm_value_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_rc_channels_pos = 			LOCATION_HIDDEN;

	_msp.Send(MSP_OSD_CONFIG, &msp_osd_config);
}

// extract it to MSPOSD_BF_Run() and MSPOSD_DJIFPV_Run() for compatibility?
void MspOsd::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		parameters_update();

		// Send updated config to OSD when parameters change
		SendConfig();
	}

	// perform first time initialization, if needed
	if (!_is_initialized) {
		struct termios t;
		_msp_fd = open(_device, O_RDWR | O_NONBLOCK);

		if (_msp_fd < 0) {
			_performance_data.initialization_problems = true;
			return;
		}

		tcgetattr(_msp_fd, &t);
		cfsetspeed(&t, B115200);
		t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
		t.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
		t.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
		t.c_oflag = 0;
		tcsetattr(_msp_fd, TCSANOW, &t);

		_msp = MspV1(_msp_fd);

		_is_initialized = true;

		// Send initial config to OSD
		SendConfig();
	}

	if (change_channel) {
		msp_get_vtx_config_t vtx_set_config{0};
		vtx_set_config.low_power_disarm = vtx_config.low_power_disarm;
		vtx_set_config.pit_mode = vtx_config.pit_mode;
		vtx_set_config.vtx_type = VTXDEV_MSP;
		vtx_set_config.band = vtx_config.user_band;
		vtx_set_config.channel = vtx_config.user_channel;
		this->Send(MSP_GET_VTX_CONFIG, &vtx_set_config, sizeof(msp_get_vtx_config_t));
		change_channel = false;
	}

	this->Receive();

	if (!has_vtx_config) {
		this->Send(MSP_GET_VTX_CONFIG, nullptr, 0);
	}

	// Always send heartbeat and clear screen, even if all symbols disabled
	uint8_t subcmd = MSP_DP_HEARTBEAT;
	this->Send(MSP_CMD_DISPLAYPORT, &subcmd, 1);

	subcmd = MSP_DP_CLEAR_SCREEN;
	this->Send(MSP_CMD_DISPLAYPORT, &subcmd, 1);

	// Skip telemetry processing if all symbols disabled
	if (_param_osd_symbols.get() == 0) {
		// Just send DRAW_SCREEN to clear the OSD
		subcmd = MSP_DP_DRAW_SCREEN;
		this->Send(MSP_CMD_DISPLAYPORT, &subcmd, 1);
		return;
	}

	// Artificial Horizon (drawn FIRST so it's behind everything else)
	{
		if (enabled(SymbolIndex::ARTIFICIAL_HORIZON)) {
			vehicle_attitude_s vehicle_attitude{};
			_vehicle_attitude_sub.copy(&vehicle_attitude);

			int crosshair_y = 10 + _param_osd_ch_height.get();
			int crosshair_x = 26;
			int ladder_width = _param_osd_ah_width.get();
			int ah_mode = _param_osd_ah_mode.get();
			int ladder_side = _param_osd_ladder_side.get();
			int ladder_step = _param_osd_ladder_step.get();
			float pitch_offset = _param_osd_ah_offset.get();

			// Mode 0: Moving horizon (classic - old style, no ladder)
			if (ah_mode == 0) {
				msp_rendor_horizon_bar_t horizon_bar{};
				msp_rendor_horizon_cursor_t cursor_left{};
				msp_rendor_horizon_cursor_t cursor_right{};

				msp_osd::construct_rendor_ARTIFICIAL_HORIZON(vehicle_attitude, crosshair_y, crosshair_x,
					ladder_width, pitch_offset, horizon_bar, cursor_left, cursor_right);

				this->Send(MSP_CMD_DISPLAYPORT, &horizon_bar, calc_rendor_size(horizon_bar));
				this->Send(MSP_CMD_DISPLAYPORT, &cursor_left, sizeof(cursor_left));
				this->Send(MSP_CMD_DISPLAYPORT, &cursor_right, sizeof(cursor_right));
			}
			// Mode 1: Fixed horizon (ladder configured via OSD_LADDER_SIDE)
			else {
				const int max_ladder_lines = 25;
				msp_rendor_pitch_ladder_line_t ladder_lines[max_ladder_lines];

				// Use ladder_side param (1=left, 2=right, 3=both)
				int num_lines = msp_osd::construct_rendor_PITCH_LADDER(
					vehicle_attitude,
					crosshair_y,
					crosshair_x,
					ladder_width,
					ladder_side,
					ladder_step,
					pitch_offset,
					ladder_lines,
					max_ladder_lines
				);

				// Send all pitch ladder lines
				for (int i = 0; i < num_lines; i++) {
					this->Send(MSP_CMD_DISPLAYPORT, &ladder_lines[i], calc_rendor_size(ladder_lines[i]));
				}
			}
		}
	}

// update display message (craft name)
	if (enabled(SymbolIndex::CRAFT_NAME)) {
		// Craft name is hardcoded (PX4 doesn't support string parameters)
		msp_name_t craft_name{};
		snprintf(craft_name.craft_name, sizeof(craft_name.craft_name), "HORNET");

		char msg[sizeof(msp_name_t) + 5] = {0};
		int index = 0;
		msg[index++] = MSP_DP_WRITE_STRING;
		msg[index++] = _param_osd_msg_y.get(); // Y position from parameter
		msg[index++] = _param_osd_msg_x.get(); // X position from parameter
		msg[index++] = 0;		// Icon attr
		msg[index++] = 0x00; // No icon
		memcpy(&msg[index], &craft_name, sizeof(msp_name_t));
		this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msg));
	}

	// MSP_FC_VARIANT
	{
		const auto msg = msp_osd::construct_FC_VARIANT();
		this->Send(MSP_FC_VARIANT, &msg, sizeof(msg));
	}

	// MSP_ANALOG
	{
		if (enabled(SymbolIndex::RSSI_VALUE)) {
			rc_channels_s rc_channels{};
			_rc_channels_sub.copy(&rc_channels);
			auto msg = msp_osd::construct_rendor_RSSI(rc_channels);
			// DisplayPort uses direct X/Y coordinates (not encoded)
			msg.screenXPosition = _param_osd_rssi_x.get();
			msg.screenYPosition = _param_osd_rssi_y.get();
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		if (enabled(SymbolIndex::THROTTLE)) {
			input_rc_s input_rc{};
			_input_rc_sub.copy(&input_rc);
			auto msg = msp_osd::construct_rendor_THROTTLE(input_rc);
			set_displayport_position(msg, _param_osd_thr_x.get(), _param_osd_thr_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// MSP_BATTERY_STATE
	{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		// Send MSP_BATTERY_STATE (for DJI compatibility)
		const auto msg_original = msp_osd::construct_BATTERY_STATE(battery_status);
		this->Send(MSP_BATTERY_STATE, &msg_original);
	}

	// MSP_RAW_GPS
	{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		if (enabled(SymbolIndex::GPS_LAT)) {
			auto msg = msp_osd::construct_rendor_GPS_LAT(vehicle_gps_position);
			set_displayport_position(msg, _param_osd_gpslat_x.get(), _param_osd_gpslat_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		if (enabled(SymbolIndex::GPS_LON)) {
			auto msg = msp_osd::construct_rendor_GPS_LON(vehicle_gps_position);
			set_displayport_position(msg, _param_osd_gpslon_x.get(), _param_osd_gpslon_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		if (enabled(SymbolIndex::GPS_SATS)) {
			auto msg = msp_osd::construct_rendor_GPS_NUM(vehicle_gps_position);
			set_displayport_position(msg, _param_osd_sats_x.get(), _param_osd_sats_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// MSP_COMP_GPS
	{
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);

		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);

		if (enabled(SymbolIndex::HOME_DIST)) {
			auto msg =  msp_osd::construct_rendor_distanceToHome(home_position, vehicle_global_position);
			set_displayport_position(msg, _param_osd_hdist_x.get(), _param_osd_hdist_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// MSP_ATTITUDE
	{
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		if (enabled(SymbolIndex::PITCH_ANGLE)) {
			auto msg = msp_osd::construct_rendor_PITCH(vehicle_attitude);
			set_displayport_position(msg, _param_osd_pitch_x.get(), _param_osd_pitch_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
		if (enabled(SymbolIndex::ROLL_ANGLE)) {
			auto msg = msp_osd::construct_rendor_ROLL(vehicle_attitude);
			set_displayport_position(msg, _param_osd_roll_x.get(), _param_osd_roll_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		// Horizon sidebars (left and right bars showing roll)
		if (enabled(SymbolIndex::HORIZON_SIDEBARS)) {
			msp_rendor_horizon_sidebar_t left, right;
			msp_osd::construct_rendor_HORIZON_SIDEBARS(vehicle_attitude, left, right);

			// Left sidebar
			left.screenXPosition = _param_osd_horiz_l_x.get();
			this->Send(MSP_CMD_DISPLAYPORT, &left, calc_rendor_size(left));

			// Right sidebar
			right.screenXPosition = _param_osd_horiz_r_x.get();
			this->Send(MSP_CMD_DISPLAYPORT, &right, calc_rendor_size(right));
		}
	}


	// MSP_ALTITUDE
	{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);

		if (enabled(SymbolIndex::ALTITUDE)) {
			auto msg = msp_osd::construct_Rendor_ALTITUDE(vehicle_gps_position, vehicle_local_position);
			set_displayport_position(msg, _param_osd_alt_x.get(), _param_osd_alt_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		// Vertical Speed
		if (enabled(SymbolIndex::NUMERICAL_VARIO)) {
			auto msg = msp_osd::construct_rendor_VSPEED(vehicle_local_position);
			set_displayport_position(msg, _param_osd_vario_x.get(), _param_osd_vario_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Additional OSD Elements - GPS Speed
	{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		if (enabled(SymbolIndex::GPS_SPEED)) {
			auto msg = msp_osd::construct_rendor_GPS_SPEED(vehicle_gps_position, _param_osd_speed_unit.get(), _param_osd_gps_spdtype.get());
			set_displayport_position(msg, _param_osd_gspd_x.get(), _param_osd_gspd_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Airspeed and Stall Warning
	{
		airspeed_validated_s airspeed_validated{};
		_airspeed_validated_sub.copy(&airspeed_validated);

		if (enabled(SymbolIndex::AIRSPEED)) {
			auto msg = msp_osd::construct_rendor_AIRSPEED(airspeed_validated, _param_osd_speed_unit.get(), _param_osd_aspd_src.get());
			set_displayport_position(msg, _param_osd_aspd_x.get(), _param_osd_aspd_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		// Max Altitude Warning - Flash orange when above OSD_MAX_ALT
		if (enabled(SymbolIndex::ALT_MAX_WARNING)) {
			vehicle_local_position_s vehicle_local_position{};
			_vehicle_local_position_sub.copy(&vehicle_local_position);
			float current_alt = vehicle_local_position.z_valid ? (-vehicle_local_position.z) : 0.0f;
			auto msg = msp_osd::construct_rendor_ALT_MAX_WARNING(current_alt, (float)_param_osd_max_alt.get(), _param_osd_max_alt_test.get() == 1);
			set_displayport_position(msg, _param_osd_max_alt_x.get(), _param_osd_max_alt_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		// Stall Warning - Flash when below FW_AIRSPD_MIN (or test mode)
		if (enabled(SymbolIndex::STALL_WARNING)) {
			auto msg = msp_osd::construct_rendor_STALL_WARNING(airspeed_validated, _param_fw_airspd_min.get(), _param_osd_stall_test.get() == 1);
			set_displayport_position(msg, _param_osd_stall_x.get(), _param_osd_stall_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		// G-Meter - Display G-force with dynamic color
		if (enabled(SymbolIndex::G_METER)) {
			sensor_combined_s sensor_combined{};
			_sensor_combined_sub.copy(&sensor_combined);
			auto msg = msp_osd::construct_rendor_G_METER(sensor_combined, _param_osd_max_g.get());
			set_displayport_position(msg, _param_osd_gmeter_x.get(), _param_osd_gmeter_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		// Pull Potential - Percentage of available load factor used at current TAS
		if (enabled(SymbolIndex::PULL_POT)) {
			airspeed_validated_s pullpot_airspeed = airspeed_validated;
			sensor_combined_s sensor_combined{};
			_sensor_combined_sub.copy(&sensor_combined);
			const int32_t source = _param_osd_pullpot_src.get();
			const bool test_mode = (_param_osd_pullpot_test.get() == 1);
			float csv_gmax = NAN;

			if (test_mode) {
				const float now_s = static_cast<float>(hrt_absolute_time()) * 1e-6f;
				pullpot_airspeed.true_airspeed_m_s = 50.0f + 50.0f * sinf(now_s * 0.35f);
				sensor_combined.accelerometer_m_s2[0] = 0.0f;
				sensor_combined.accelerometer_m_s2[1] = 0.0f;
				sensor_combined.accelerometer_m_s2[2] = 3.0f * 9.80665f;
			}

			if (source == PULLPOT_SOURCE_CSV_SD && _pullpot_csv_loaded) {
				csv_gmax = pullpot_csv_gmax_for_tas(pullpot_airspeed.true_airspeed_m_s);
			}

			auto msg = msp_osd::construct_rendor_PULL_POT(
				pullpot_airspeed,
				sensor_combined,
				source,
				_param_fw_airspd_stall.get(),
				csv_gmax
			);
			set_displayport_position(msg, _param_osd_pullpot_x.get(), _param_osd_pullpot_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Home Direction
	{
		home_position_s home_position{};
		_home_position_sub.copy(&home_position);

		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);

		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		if (enabled(SymbolIndex::HOME_DIR)) {
			auto msg = msp_osd::construct_rendor_HOME_DIR(home_position, vehicle_global_position, vehicle_attitude);
			set_displayport_position(msg, _param_osd_hdir_x.get(), _param_osd_hdir_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, sizeof(msp_rendor_home_direction_t)); // Icon-only
		}
	}

	// Power, Cell Voltage, and Battery Voltage
	{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		if (enabled(SymbolIndex::POWER)) {
			auto msg = msp_osd::construct_rendor_POWER(battery_status);
			set_displayport_position(msg, _param_osd_power_x.get(), _param_osd_power_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}

		if (enabled(SymbolIndex::MAIN_BATT_VOLTAGE)) {
			auto msg = msp_osd::construct_rendor_BATTERY_STATE(battery_status);
			set_displayport_position(msg, _param_osd_batvolt_x.get(), _param_osd_batvolt_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Flight Mode
	{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (enabled(SymbolIndex::FLYMODE)) {
			bool use_short = (_param_osd_fmode_size.get() == 0);
			auto msg = msp_osd::construct_rendor_FLIGHT_MODE(vehicle_status, use_short);
			set_displayport_position(msg, _param_osd_flymode_x.get(), _param_osd_flymode_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Crosshairs (3 symbols centered at 26: positions 25, 26, 27)
	{
		if (enabled(SymbolIndex::CROSSHAIRS)) {
			auto msg = msp_osd::construct_rendor_CROSSHAIRS();
			set_displayport_position(msg, 25, 10 + _param_osd_ch_height.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Heading (2 rows)
	{
		if (enabled(SymbolIndex::HEADING)) {
			vehicle_attitude_s vehicle_attitude{};
			_vehicle_attitude_sub.copy(&vehicle_attitude);

			home_position_s home_position{};
			_home_position_sub.copy(&home_position);

			vehicle_global_position_s vehicle_global_position{};
			_vehicle_global_position_sub.copy(&vehicle_global_position);

			// Get heading bars (fills both row1 and row2)
			msp_rendor_heading_t row1{};
			msp_rendor_heading_t row2{};
			msp_osd::construct_rendor_HEADING(vehicle_attitude, home_position, vehicle_global_position, row1, row2);

			// Send top row (cardinal directions with arrow)
			set_displayport_position(row1, _param_osd_heading_x.get(), _param_osd_heading_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &row1, calc_rendor_size(row1));

			// Send bottom row (bar with ticks and arrow)
			set_displayport_position(row2, _param_osd_heading_x.get(), _param_osd_heading_y.get() + 1);
			this->Send(MSP_CMD_DISPLAYPORT, &row2, calc_rendor_size(row2));
		}
	}

	// Arming status (standalone DISARMED/ARMED indicator)
	{
		if (enabled(SymbolIndex::DISARMED)) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);
			auto msg = msp_osd::construct_rendor_ARMING(vehicle_status);
			set_displayport_position(msg, _param_osd_disarm_x.get(), _param_osd_disarm_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Warning messages (standalone)
	{
		if (enabled(SymbolIndex::WARNING)) {
			log_message_s log_message{};
			_log_message_sub.copy(&log_message);

			uint8_t log_level = log_message.severity;
			uint8_t osd_log_level = _param_osd_log_level.get();

			// Filter: only show messages with severity <= OSD_LOG_LEVEL
			// (0=EMERG is most critical, 7=DEBUG is least)
			if (log_level <= osd_log_level) {
				// Message passes filter, process and display
			if (log_level != _last_log_level || log_message.text[0] != '\0') {
				// Get severity prefix
				const char *prefix;
				switch (log_level) {
				case 0: prefix = "EMERG: "; break;
				case 1: prefix = "ALERT: "; break;
				case 2: prefix = "CRIT: "; break;
				case 3: prefix = "ERROR: "; break;
				case 4: prefix = "WARN: "; break;
				case 5: prefix = "NOTICE: "; break;
				case 6: prefix = "INFO: "; break;
				case 7: prefix = "DEBUG: "; break;
				default: prefix = ""; break;
				}

				// Store prefix for display
				strncpy(_last_warning_prefix, prefix, sizeof(_last_warning_prefix) - 1);
				_last_warning_prefix[sizeof(_last_warning_prefix) - 1] = '\0';

				// Copy and sanitize text (Betaflight uses 0x60-0xFF for glyphs!)
				char sanitized_text[128];
				int dst_idx = 0;
				for (int i = 0; i < (int)sizeof(log_message.text) && log_message.text[i] != '\0' && dst_idx < 127; i++) {
					unsigned char c = (unsigned char)log_message.text[i];

					// Betaflight ASCII safe zone: 0x20-0x5F only
					if (c >= 0x20 && c <= 0x5F) {
						// Safe ASCII (space to underscore)
						sanitized_text[dst_idx++] = c;
					} else if (c >= 0x61 && c <= 0x7A) {
						// Lowercase a-z → UPPERCASE (avoid glyphs 0x60-0x7F)
						sanitized_text[dst_idx++] = c - 0x20;  // 'a' → 'A'
					}
					// Ignore 0x00-0x1F (control chars) and 0x80-0xFF (glyphs)
				}
				sanitized_text[dst_idx] = '\0';

				// Set sanitized message text for scrolling
				_warning_display.set_message(sanitized_text);
				_last_log_level = log_level;
				_last_warning_time = hrt_absolute_time();
			}

			// Clear after 30 seconds
			if ((hrt_absolute_time() - _last_warning_time) > 30_s) {
				_warning_display.set_message("");
				_last_log_level = UINT8_MAX;
				_last_warning_prefix[0] = '\0';
			}

			// Build display: fixed prefix + scrolled text
			msp_rendor_warning_t msg{};
			memset(msg.str, 0, sizeof(msg.str)); // Zero-init to avoid garbage

			int prefix_len = strlen(_last_warning_prefix);
			int text_space = 30 - prefix_len;  // Remaining space for scrolled text

			if (text_space > 0) {
				char scrolled_text[31];
				_warning_display.get(scrolled_text, text_space, hrt_absolute_time());
				snprintf(msg.str, sizeof(msg.str), "%s%s", _last_warning_prefix, scrolled_text);
			} else {
				// Prefix too long, just show prefix
				strncpy(msg.str, _last_warning_prefix, sizeof(msg.str) - 1);
			}
			msg.str[sizeof(msg.str) - 1] = '\0';

			// CRITICAL: Force all bytes to 0x00-0x7F to avoid Betaflight glyphs
			for (int i = 0; i < (int)sizeof(msg.str); i++) {
				msg.str[i] = msg.str[i] & 0x7F;  // Mask high bit
			}

			msg.str_length = strlen(msg.str);

			// Color based on severity (0x00=white, 0x02=orange, 0x03=red)
			if (log_level <= 2) {
				msg.iconAttrs = 0x03; // Red (EMERG, ALERT, CRIT)
			} else if (log_level <= 4) {
				msg.iconAttrs = 0x02; // Orange (ERROR, WARN)
			} else {
				msg.iconAttrs = 0x00; // White (NOTICE, INFO, DEBUG)
			}

			set_displayport_position(msg, _param_osd_warn_x.get(), _param_osd_warn_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
			}
		}
	}

	// ESC Temperature
	{
		if (enabled(SymbolIndex::ESC_TMP)) {
			esc_status_s esc_status{};
			_esc_status_sub.copy(&esc_status);
			auto msg = msp_osd::construct_rendor_ESC_TEMP(esc_status);
			set_displayport_position(msg, _param_osd_esc_x.get(), _param_osd_esc_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// ESC RPM
	{
		if (enabled(SymbolIndex::ESC_RPM)) {
			esc_status_s esc_status{};
			_esc_status_sub.copy(&esc_status);
			auto msg = msp_osd::construct_rendor_ESC_RPM(esc_status);
			set_displayport_position(msg, _param_osd_esc_rpm_x.get(), _param_osd_esc_rpm_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// ESC Current (DisplayPort)
	{
		if (enabled(SymbolIndex::ESC_AMP)) {
			esc_status_s esc_status{};
			_esc_status_sub.copy(&esc_status);
			auto msg = msp_osd::construct_rendor_ESC_AMP(esc_status);
			set_displayport_position(msg, _param_osd_esc_amp_x.get(), _param_osd_esc_amp_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// Current Draw (DisplayPort)
	{
		if (enabled(SymbolIndex::CURRENT_DRAW)) {
			battery_status_s battery_status{};
			_battery_status_sub.copy(&battery_status);
			auto msg = msp_osd::construct_rendor_CURRENT(battery_status);
			set_displayport_position(msg, _param_osd_current_x.get(), _param_osd_current_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// MAH Drawn (DisplayPort)
	{
		if (enabled(SymbolIndex::MAH_DRAWN)) {
			battery_status_s battery_status{};
			_battery_status_sub.copy(&battery_status);
			auto msg = msp_osd::construct_rendor_MAH(battery_status);
			set_displayport_position(msg, _param_osd_mah_x.get(), _param_osd_mah_y.get());
			this->Send(MSP_CMD_DISPLAYPORT, &msg, calc_rendor_size(msg));
		}
	}

	// MSP_MOTOR_TELEMETRY
	{

	}

	// MSP_RC
	{
		if (_param_osd_rc_stick.get() == 1) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
				input_rc_s input_rc{};
				_input_rc_sub.copy(&input_rc);
				const auto msg = msp_osd::construct_MSP_RC(input_rc);
				this->Send(MSP_RC, &msg, sizeof(msp_rc_t));
			}
		}

	}

	// MSP_STATUS
	{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		const auto msg = msp_osd::construct_MSP_STATUS(vehicle_status);
		this->Send(MSP_STATUS, &msg, sizeof(msp_status_t));
	}

	subcmd = MSP_DP_DRAW_SCREEN;
	this->Send(MSP_CMD_DISPLAYPORT, &subcmd, 1);
}

void MspOsd::Send(const unsigned int message_type, const void *payload)
{
	if (_msp.Send(message_type, payload)) {
		_performance_data.successful_sends++;

	} else {
		_performance_data.unsuccessful_sends++;
	}
}
void MspOsd::Send(const unsigned int message_type, const void *payload, int32_t payload_size)
{
	if (_msp.Send(message_type, payload, payload_size)) {
		_performance_data.successful_sends++;

	} else {
		_performance_data.unsuccessful_sends++;
	}
}

void MspOsd::Receive()
{
	uint8_t packet[255];
	uint8_t message_id;
	int ret;

	while ((ret = _msp.Receive(packet, &message_id)) != -EWOULDBLOCK) {
		if (ret >= 0) {
			switch (message_id) {

			case MSP_SET_VTX_CONFIG: {
					if (ret == 0xF) {
						memcpy((void *)&vtx_config, packet, sizeof(vtx_config));
						has_vtx_config = true;
					}

					break;
				}

			case MSP_SET_VTXTABLE_BAND: {
					msp_set_vtxtable_band_t *band_info = (msp_set_vtxtable_band_t *)&packet[0];

					// Only supported fixed name lenght and < 8 channels for now
					if (band_info->band <= BAND_COUNT && band_info->band_name_length == 8 && band_info->channel_count <= 8) {
						memcpy((void *)&vtx_bands[band_info->band - 1], packet, sizeof(msp_set_vtxtable_band_t));

						if (has_vtx_config && band_info->band == vtx_config.band_count) {
							has_vtx_bands = true;
						}
					}

					break;
				}

			case MSP_SET_VTXTABLE_POWERLEVEL: {
					if ((packet[0] - 1) < POWER_LEVEL_COUNT) {
						memcpy((void *)&power_levels[packet[0] - 1], packet, sizeof(msp_set_vtxtable_powerlevel_t));
						has_power_config = true;
					}

					break;
				}

			default:
				break;

			}
		}
	}

}

void MspOsd::load_pullpot_csv()
{
	const char *csv_path = "/fs/microsd/vn_limits.csv";
	FILE *fp = fopen(csv_path, "r");

	_pullpot_vn_points_count = 0;
	_pullpot_csv_loaded = false;

	if (fp == nullptr) {
		PX4_WARN("PULL_POT: csv not found: %s", csv_path);
		return;
	}

	char line[128]{};

	while (fgets(line, sizeof(line), fp) != nullptr) {
		float tas = NAN;
		float gmax = NAN;

		int parsed = sscanf(line, " %f , %f", &tas, &gmax);

		if (parsed != 2) {
			parsed = sscanf(line, " %f ; %f", &tas, &gmax);
		}

		if (parsed != 2 || !PX4_ISFINITE(tas) || !PX4_ISFINITE(gmax) || tas <= 0.0f || gmax <= 0.0f) {
			continue;
		}

		if (_pullpot_vn_points_count >= PULLPOT_VN_MAX_POINTS) {
			break;
		}

		int insert_at = _pullpot_vn_points_count;

		for (int i = 0; i < _pullpot_vn_points_count; i++) {
			if (tas < _pullpot_vn_points[i].tas_m_s) {
				insert_at = i;
				break;
			}

			if (fabsf(tas - _pullpot_vn_points[i].tas_m_s) < 0.001f) {
				_pullpot_vn_points[i].g_max = gmax;
				insert_at = -1;
				break;
			}
		}

		if (insert_at >= 0) {
			for (int j = _pullpot_vn_points_count; j > insert_at; j--) {
				_pullpot_vn_points[j] = _pullpot_vn_points[j - 1];
			}

			_pullpot_vn_points[insert_at].tas_m_s = tas;
			_pullpot_vn_points[insert_at].g_max = gmax;
			_pullpot_vn_points_count++;
		}
	}

	fclose(fp);

	if (_pullpot_vn_points_count >= 2) {
		_pullpot_csv_loaded = true;
		PX4_INFO("PULL_POT: loaded %d CSV points", _pullpot_vn_points_count);
	} else {
		PX4_WARN("PULL_POT: not enough CSV points (%d)", _pullpot_vn_points_count);
	}
}

float MspOsd::pullpot_csv_gmax_for_tas(float tas_m_s) const
{
	if (!_pullpot_csv_loaded || _pullpot_vn_points_count <= 0 || !PX4_ISFINITE(tas_m_s)) {
		return NAN;
	}

	if (tas_m_s <= _pullpot_vn_points[0].tas_m_s) {
		return _pullpot_vn_points[0].g_max;
	}

	if (tas_m_s >= _pullpot_vn_points[_pullpot_vn_points_count - 1].tas_m_s) {
		return _pullpot_vn_points[_pullpot_vn_points_count - 1].g_max;
	}

	for (int i = 0; i < _pullpot_vn_points_count - 1; i++) {
		const auto &p0 = _pullpot_vn_points[i];
		const auto &p1 = _pullpot_vn_points[i + 1];

		if (tas_m_s >= p0.tas_m_s && tas_m_s <= p1.tas_m_s) {
			const float dt = p1.tas_m_s - p0.tas_m_s;

			if (dt <= 0.0f) {
				return p0.g_max;
			}

			const float alpha = (tas_m_s - p0.tas_m_s) / dt;
			return p0.g_max + alpha * (p1.g_max - p0.g_max);
		}
	}

	return _pullpot_vn_points[_pullpot_vn_points_count - 1].g_max;
}

void MspOsd::parameters_update()
{
	// update our display rate and dwell time for warning display
	_warning_display.set_period(hrt_abstime(_param_osd_scroll_rate.get() * 1000ULL));
	_warning_display.set_dwell(hrt_abstime(_param_osd_dwell_time.get() * 1000ULL));

	const int32_t source = _param_osd_pullpot_src.get();

	if (source == PULLPOT_SOURCE_CSV_SD
	    && (!_pullpot_csv_loaded || _last_pullpot_source != PULLPOT_SOURCE_CSV_SD)) {
		load_pullpot_csv();
	}

	_last_pullpot_source = source;
}

bool MspOsd::enabled(const SymbolIndex &symbol)
{
	return _param_osd_symbols.get() & (1u << symbol);
}

int MspOsd::task_spawn(int argc, char *argv[])
{
	// initialize device
	const char *device = nullptr;
	bool error_flag = false;

	// loop through input arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device) {
		PX4_ERR("Missing device");
		return PX4_ERROR;
	}

	MspOsd *instance = new MspOsd(device);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MspOsd::print_status()
{
	PX4_INFO("Running on %s", _device);
	PX4_INFO("\tinitialized: %d", _is_initialized);
	PX4_INFO("\tinitialization issues: %d", _performance_data.initialization_problems);
	PX4_INFO("\tscroll rate: %d", static_cast<int>(_param_osd_scroll_rate.get()));
	PX4_INFO("\tsuccessful sends: %lu", _performance_data.successful_sends);
	PX4_INFO("\tunsuccessful sends: %lu", _performance_data.unsuccessful_sends);

	// print current warning message
	char msg[31];
	_warning_display.get(msg, 30, hrt_absolute_time());
	PX4_INFO("Current warning: \n\t%s", msg);

	if (has_vtx_config) {
		PX4_INFO("=== VTX Configuration ===");

		if (has_vtx_bands) {
			PX4_INFO("Channel: %c%u", vtx_bands[vtx_config.user_band - 1].band_letter, vtx_config.user_channel);

		} else {
			PX4_INFO("Band: %u", vtx_config.user_band);
			PX4_INFO("Channel: %u", vtx_config.user_channel);
		}

		PX4_INFO("Frequency: %u MHz", vtx_config.user_freq);

		if (has_power_config && (vtx_config.power_level - 1) < POWER_LEVEL_COUNT) {
			PX4_INFO("Transmit power: %.*s mW", power_levels[vtx_config.power_level - 1].power_label_length,
				 power_levels[vtx_config.power_level - 1].power_label_name);

		} else {
			PX4_INFO("Power Level: %u/%u", vtx_config.power_level,  vtx_config.power_count);

		}

		PX4_INFO("PIT Mode: %s", vtx_config.pit_mode ? "On" : "Off");

		const char *disarm_modes[] = {
			"Off",
			"Always",
			"Until First Arm"
		};

		if (vtx_config.low_power_disarm < 3) {
			PX4_INFO("Low Power Disarm: %s", disarm_modes[vtx_config.low_power_disarm]);

		} else {
			PX4_INFO("Low Power Disarm: Unknown (%u)", vtx_config.low_power_disarm);
		}

		PX4_INFO("PIT Frequency: %u MHz", vtx_config.pit_freq);

	} else {
		PX4_INFO("No VTX Configuration available, can't do channel switching");
	}

	if (has_vtx_config) {
		PX4_INFO("=== VTX Configuration ===");

		if (has_vtx_bands) {
			PX4_INFO("Channel: %c%u", vtx_bands[vtx_config.user_band - 1].band_letter, vtx_config.user_channel);

		} else {
			PX4_INFO("Band: %u", vtx_config.user_band);
			PX4_INFO("Channel: %u", vtx_config.user_channel);
		}

		PX4_INFO("Frequency: %u MHz", vtx_config.user_freq);

		if (has_power_config && (vtx_config.power_level - 1) < POWER_LEVEL_COUNT) {
			PX4_INFO("Transmit power: %.*s mW", power_levels[vtx_config.power_level - 1].power_label_length,
				 power_levels[vtx_config.power_level - 1].power_label_name);

		} else {
			PX4_INFO("Power Level: %u/%u", vtx_config.power_level,  vtx_config.power_count);

		}

		PX4_INFO("PIT Mode: %s", vtx_config.pit_mode ? "On" : "Off");

		const char *disarm_modes[] = {
			"Off",
			"Always",
			"Until First Arm"
		};

		if (vtx_config.low_power_disarm < 3) {
			PX4_INFO("Low Power Disarm: %s", disarm_modes[vtx_config.low_power_disarm]);

		} else {
			PX4_INFO("Low Power Disarm: Unknown (%u)", vtx_config.low_power_disarm);
		}

		PX4_INFO("PIT Frequency: %u MHz", vtx_config.pit_freq);

	} else {
		PX4_INFO("No VTX Configuration available, can't do channel switching");
	}

	return 0;
}

int MspOsd::set_channel(char *new_channel)
{
	char band_letter = toupper(new_channel[0]);

	if (!has_vtx_bands) {
		return -2;
	}

	for (int i = 0; i < BAND_COUNT; i++) {
		if (vtx_bands[i].band != 0) {
			if (band_letter == toupper(vtx_bands[i].band_letter)) {
				int channel = atoi(&new_channel[1]);

				if (channel > 0 && channel <= vtx_config.channel_count && vtx_bands[i].frequency[channel - 1] != 0) {
					vtx_config.user_band = vtx_bands[i].band;
					vtx_config.user_channel = channel;
					vtx_config.user_freq = vtx_bands[i].frequency[channel - 1];
					change_channel = true;
					return 0;
				}

			}
		}
	}

	return -1;
}

int MspOsd::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp("channel", argv[0]) == 0) {
		if (argc == 1) {
			PX4_INFO("Please provide a channel");

		} else if (is_running() && _object.load()) {
			MspOsd *object = _object.load();
			int ret = object->set_channel(argv[1]);

			if (ret == -1) {
				PX4_INFO("Channel not found");

			} else if (ret == -2) {
				PX4_INFO("No VTX Channel table available");
			}

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;
}

int MspOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MSP telemetry streamer

### Implementation
Converts uORB messages to MSP telemetry packets

### Examples
CLI usage example:
$ msp_osd

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("msp_osd", "driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("channel", "Change VTX channel");

	return 0;
}

extern "C" __EXPORT int msp_osd_main(int argc, char *argv[])
{
	return MspOsd::main(argc, argv);
}

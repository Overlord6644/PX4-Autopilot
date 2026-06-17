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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "MspV1.hpp"
#include "MessageDisplay/MessageDisplay.hpp"
#include "uorb_to_msp.hpp"

using namespace time_literals;

// location to "hide" unused display elements
#define LOCATION_HIDDEN 234;

#define POWER_LEVEL_COUNT 5
#define BAND_COUNT 7
#define PULLPOT_VN_MAX_POINTS 128

struct PerformanceData {
	bool initialization_problems{false};
	long unsigned int successful_sends{0};
	long unsigned int unsuccessful_sends{0};
};

// mapping from symbol name to bit in the parameter bitmask
//  @TODO investigate params; it seems like this should be available directly?
enum SymbolIndex : uint8_t {
	CRAFT_NAME		= 0,  // Hardcoded
	DISARMED		= 1,
	GPS_LAT			= 2,
	GPS_LON			= 3,
	GPS_SATS		= 4,
	GPS_SPEED		= 5,
	HOME_DIST		= 6,
	HOME_DIR		= 7,
	MAIN_BATT_VOLTAGE	= 8,
	CURRENT_DRAW		= 9,
	MAH_DRAWN		= 10,
	RSSI_VALUE		= 11,
	ALTITUDE		= 12,
	NUMERICAL_VARIO		= 13,
	FLYMODE			= 14,
	ESC_TMP			= 15,
	PITCH_ANGLE		= 16,
	ROLL_ANGLE		= 17,
	CROSSHAIRS		= 18,
	HORIZON_SIDEBARS	= 19,
	POWER			= 20,
	AIRSPEED		= 21,
	STALL_WARNING		= 22,
	HEADING			= 23,
	WARNING			= 24,
	ARTIFICIAL_HORIZON	= 25,
	ALT_MAX_WARNING		= 26,
	G_METER			= 27,
	ESC_RPM			= 28,
	ESC_AMP			= 29,
	PULL_POT			= 30,
	THROTTLE			= 31
};

class MspOsd : public ModuleBase<MspOsd>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MspOsd(const char *device);

	~MspOsd() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int set_channel(char *new_channel);

private:
	struct PullPotVnPoint {
		float tas_m_s;
		float g_max;
	};

	enum PullPotSource : int32_t {
		PULLPOT_SOURCE_FORMULA = 0,
		PULLPOT_SOURCE_CSV_SD = 1
	};

	void Run() override;

	// update a single display element in the display
	void Send(const unsigned int message_type, const void *payload);
	void Send(const unsigned int message_type, const void *payload, int32_t payload_size);

	// receive vtx data
	void Receive();

	// send full configuration to MSP (triggers the actual update)
	void SendConfig();
	void SendTelemetry();

	// perform actions required for local updates
	void parameters_update();
	void load_pullpot_csv();
	float pullpot_csv_gmax_for_tas(float tas_m_s) const;

	// convenience function to check if a given symbol is enabled
	bool enabled(const SymbolIndex &symbol);

	MspV1 _msp{0};
	int _msp_fd{-1};

	bool _is_initialized{false};

	// subscriptions to desired vehicle display information
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _log_message_sub{ORB_ID(log_message)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// local heartbeat
	bool _heartbeat{false};

	// helper function to calculate MSP OSD position from X/Y coordinates
	// Position encoding: 2048 + X + (Y * 32) - for MSP_OSD_CONFIG only
	// Returns 0 (hidden) if position would be invalid
	uint16_t calculate_position(int32_t x, int32_t y) {
		if (x < 0 || y < 0) {
			return 0;
		}
		return 2048 + x + (y * 32);
	}

	// Helper to set DisplayPort position (uses direct X/Y, not encoded)
	template<typename T>
	void set_displayport_position(T& msg, int32_t x, int32_t y) {
		if (x < 0 || y < 0) {
			msg.screenXPosition = 0;
			msg.screenYPosition = 0;
		} else {
			msg.screenXPosition = x;
			msg.screenYPosition = y;
		}
	}

	// parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_SYMBOLS>) _param_osd_symbols,
		(ParamInt<px4::params::OSD_CH_HEIGHT>) _param_osd_ch_height,
		(ParamInt<px4::params::OSD_AH_WIDTH>) _param_osd_ah_width,
		(ParamFloat<px4::params::OSD_AH_OFFSET>) _param_osd_ah_offset,
		(ParamInt<px4::params::OSD_AH_MODE>) _param_osd_ah_mode,
		(ParamInt<px4::params::OSD_LADDER_SIDE>) _param_osd_ladder_side,
		(ParamInt<px4::params::OSD_LADDER_STEP>) _param_osd_ladder_step,
		(ParamInt<px4::params::OSD_SCROLL_RATE>) _param_osd_scroll_rate,
		(ParamInt<px4::params::OSD_DWELL_TIME>) _param_osd_dwell_time,
		(ParamInt<px4::params::OSD_LOG_LEVEL>) _param_osd_log_level,
		(ParamInt<px4::params::OSD_RC_STICK>) _param_osd_rc_stick,
		(ParamInt<px4::params::OSD_SPEED_UNIT>) _param_osd_speed_unit,
		(ParamInt<px4::params::OSD_GPS_SPDTYPE>) _param_osd_gps_spdtype,
		(ParamInt<px4::params::OSD_ASPD_SRC>) _param_osd_aspd_src,
		(ParamInt<px4::params::OSD_STALL_TEST>) _param_osd_stall_test,
		// Position parameters
		(ParamInt<px4::params::OSD_GPSLAT_X>) _param_osd_gpslat_x,
		(ParamInt<px4::params::OSD_GPSLAT_Y>) _param_osd_gpslat_y,
		(ParamInt<px4::params::OSD_GPSLON_X>) _param_osd_gpslon_x,
		(ParamInt<px4::params::OSD_GPSLON_Y>) _param_osd_gpslon_y,
		(ParamInt<px4::params::OSD_SATS_X>) _param_osd_sats_x,
		(ParamInt<px4::params::OSD_SATS_Y>) _param_osd_sats_y,
		(ParamInt<px4::params::OSD_GSPD_X>) _param_osd_gspd_x,
		(ParamInt<px4::params::OSD_GSPD_Y>) _param_osd_gspd_y,
		(ParamInt<px4::params::OSD_ASPD_X>) _param_osd_aspd_x,
		(ParamInt<px4::params::OSD_ASPD_Y>) _param_osd_aspd_y,
		(ParamInt<px4::params::OSD_HDIST_X>) _param_osd_hdist_x,
		(ParamInt<px4::params::OSD_HDIST_Y>) _param_osd_hdist_y,
		(ParamInt<px4::params::OSD_HDIR_X>) _param_osd_hdir_x,
		(ParamInt<px4::params::OSD_HDIR_Y>) _param_osd_hdir_y,
		(ParamInt<px4::params::OSD_ALT_X>) _param_osd_alt_x,
		(ParamInt<px4::params::OSD_ALT_Y>) _param_osd_alt_y,
		(ParamInt<px4::params::OSD_BATVOLT_X>) _param_osd_batvolt_x,
		(ParamInt<px4::params::OSD_BATVOLT_Y>) _param_osd_batvolt_y,
		(ParamInt<px4::params::OSD_CURRENT_X>) _param_osd_current_x,
		(ParamInt<px4::params::OSD_CURRENT_Y>) _param_osd_current_y,
		(ParamInt<px4::params::OSD_MAH_X>) _param_osd_mah_x,
		(ParamInt<px4::params::OSD_MAH_Y>) _param_osd_mah_y,
		(ParamInt<px4::params::OSD_RSSI_X>) _param_osd_rssi_x,
		(ParamInt<px4::params::OSD_RSSI_Y>) _param_osd_rssi_y,
		(ParamInt<px4::params::OSD_THR_X>) _param_osd_thr_x,
		(ParamInt<px4::params::OSD_THR_Y>) _param_osd_thr_y,
		(ParamInt<px4::params::OSD_POWER_X>) _param_osd_power_x,
		(ParamInt<px4::params::OSD_POWER_Y>) _param_osd_power_y,
		(ParamInt<px4::params::OSD_MSG_X>) _param_osd_msg_x,
		(ParamInt<px4::params::OSD_MSG_Y>) _param_osd_msg_y,
		(ParamInt<px4::params::OSD_DISARM_X>) _param_osd_disarm_x,
		(ParamInt<px4::params::OSD_DISARM_Y>) _param_osd_disarm_y,
		(ParamInt<px4::params::OSD_VARIO_X>) _param_osd_vario_x,
		(ParamInt<px4::params::OSD_VARIO_Y>) _param_osd_vario_y,
		(ParamInt<px4::params::OSD_FLYMODE_X>) _param_osd_flymode_x,
		(ParamInt<px4::params::OSD_FLYMODE_Y>) _param_osd_flymode_y,
		(ParamInt<px4::params::OSD_FMODE_SIZE>) _param_osd_fmode_size,
		(ParamInt<px4::params::OSD_HEADING_X>) _param_osd_heading_x,
		(ParamInt<px4::params::OSD_HEADING_Y>) _param_osd_heading_y,
		(ParamInt<px4::params::OSD_WARN_X>) _param_osd_warn_x,
		(ParamInt<px4::params::OSD_WARN_Y>) _param_osd_warn_y,
		(ParamInt<px4::params::OSD_ESC_X>) _param_osd_esc_x,
		(ParamInt<px4::params::OSD_ESC_Y>) _param_osd_esc_y,
		(ParamInt<px4::params::OSD_ESC_RPM_X>) _param_osd_esc_rpm_x,
		(ParamInt<px4::params::OSD_ESC_RPM_Y>) _param_osd_esc_rpm_y,
		(ParamInt<px4::params::OSD_ESC_AMP_X>) _param_osd_esc_amp_x,
		(ParamInt<px4::params::OSD_ESC_AMP_Y>) _param_osd_esc_amp_y,
		(ParamInt<px4::params::OSD_HORIZ_L_X>) _param_osd_horiz_l_x,
		(ParamInt<px4::params::OSD_HORIZ_R_X>) _param_osd_horiz_r_x,
		(ParamInt<px4::params::OSD_PITCH_X>) _param_osd_pitch_x,
		(ParamInt<px4::params::OSD_PITCH_Y>) _param_osd_pitch_y,
		(ParamInt<px4::params::OSD_ROLL_X>) _param_osd_roll_x,
		(ParamInt<px4::params::OSD_ROLL_Y>) _param_osd_roll_y,
		(ParamInt<px4::params::OSD_STALL_X>) _param_osd_stall_x,
		(ParamInt<px4::params::OSD_STALL_Y>) _param_osd_stall_y,
		(ParamInt<px4::params::OSD_MAX_ALT>) _param_osd_max_alt,
		(ParamInt<px4::params::OSD_MAX_ALT_TEST>) _param_osd_max_alt_test,
		(ParamInt<px4::params::OSD_MAX_ALT_X>) _param_osd_max_alt_x,
		(ParamInt<px4::params::OSD_MAX_ALT_Y>) _param_osd_max_alt_y,
		(ParamFloat<px4::params::OSD_MAX_G>) _param_osd_max_g,
		(ParamInt<px4::params::OSD_PULLPOT_SRC>) _param_osd_pullpot_src,
		(ParamInt<px4::params::OSD_PULLPOT_TEST>) _param_osd_pullpot_test,
		(ParamInt<px4::params::OSD_GMETER_X>) _param_osd_gmeter_x,
		(ParamInt<px4::params::OSD_GMETER_Y>) _param_osd_gmeter_y,
		(ParamInt<px4::params::OSD_PULLPOT_X>) _param_osd_pullpot_x,
		(ParamInt<px4::params::OSD_PULLPOT_Y>) _param_osd_pullpot_y,
		(ParamFloat<px4::params::FW_AIRSPD_STALL>) _param_fw_airspd_stall,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min
	)

	// metadata
	char _device[64] {};
	PerformanceData _performance_data{};

	// Warning message display
	msp_osd::MessageDisplay _warning_display{};
	uint8_t _last_log_level{UINT8_MAX};
	uint64_t _last_warning_time{0};
	char _last_warning_prefix[9]{""};  // Max 8 chars + null terminator
	PullPotVnPoint _pullpot_vn_points[PULLPOT_VN_MAX_POINTS] {};
	int _pullpot_vn_points_count{0};
	bool _pullpot_csv_loaded{false};
	int32_t _last_pullpot_source{-1};

	msp_set_vtx_config_t vtx_config;
	msp_set_vtxtable_powerlevel_t power_levels[POWER_LEVEL_COUNT];
	msp_set_vtxtable_band_t vtx_bands[BAND_COUNT] {};
	bool has_vtx_config {false};
	bool has_power_config {false};
	bool has_vtx_bands {false};
	bool change_channel {false};
};

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

/* uorb_to_msp.cpp
 *
 * Implementation file for UORB -> MSP conversion functions.
 */

// includes for mathematical manipulation
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/modes/ui.hpp>
#include <matrix/math.hpp>

// clock access
#include <px4_platform_common/defines.h>
using namespace time_literals;

#include "uorb_to_msp.hpp"

namespace msp_osd
{
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

msp_fc_variant_t construct_FC_VARIANT()
{
	// initialize result
	msp_fc_variant_t variant{};

	memcpy(variant.flightControlIdentifier, "BTFL", sizeof(variant.flightControlIdentifier));
	return variant;
}

msp_status_BF_t construct_STATUS(const vehicle_status_s &vehicle_status)
{
	// initialize result
	msp_status_BF_t status_BF = {0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status_BF.flight_mode_flags |= ARM_ACRO_BF;

		switch (vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			status_BF.flight_mode_flags |= 0;
			break;

		case vehicle_status_s::NAVIGATION_STATE_STAB:
			status_BF.flight_mode_flags |= STAB_BF;
			break;

		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			status_BF.flight_mode_flags |= RESC_BF;
			break;

		case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			status_BF.flight_mode_flags |= FS_BF;
			break;

		default:
			status_BF.flight_mode_flags |= 0;
			break;
		}
	}

	status_BF.arming_disable_flags_count = 1;
	status_BF.arming_disable_flags  = !(vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	return status_BF;
}

msp_analog_t construct_ANALOG(const battery_status_s &battery_status, const input_rc_s &input_rc)
{
	// initialize result
	msp_analog_t analog {0};

	analog.vbat = battery_status.voltage_v * 10; // bottom right... v * 10
	analog.rssi = (uint16_t)((input_rc.link_quality * 1023.0f) / 100.0f);
	analog.amperage = battery_status.current_a * 100; // main amperage
	analog.mAhDrawn = battery_status.discharged_mah; // unused
	return analog;
}

msp_rendor_rssi_t construct_rendor_RSSI(const rc_channels_s &rc_channels)
{
	msp_rendor_rssi_t rssi;
	// Position will be set by caller
	rssi.screenYPosition = 0;
	rssi.screenXPosition = 0;

	// rc_channels.rssi is already 0-100% (copied from input_rc.rssi with RSSI_MAX=100)
	int rssi_value;
	if (rc_channels.timestamp != 0 && !rc_channels.signal_lost) {
		// Valid RC signal, use rssi directly (already 0-100%)
		rssi_value = rc_channels.rssi;
	} else {
		rssi_value = 0; // No signal
	}

	snprintf(&rssi.str[0], sizeof(rssi.str), "%3d", rssi_value);
	rssi.str[3] = '%';
	rssi.str_length = 4; // "100%" always 4 chars

	return rssi;
}

msp_battery_state_t construct_BATTERY_STATE(const battery_status_s &battery_status)
{
	// initialize result
	msp_battery_state_t battery_state = {0};

	// MSP_BATTERY_STATE
	battery_state.amperage = battery_status.current_a * 100.0f; // Used for power element
	battery_state.batteryVoltage = (uint16_t)((battery_status.voltage_v / battery_status.cell_count) * 400.0f);  // OK
	battery_state.mAhDrawn = battery_status.discharged_mah ; // OK
	battery_state.batteryCellCount = battery_status.cell_count;
	battery_state.batteryCapacity = battery_status.capacity; // not used?

	// Voltage color 0==white, 1==red
	if (battery_status.voltage_v < 14.4f) {
		battery_state.batteryState = 1;

	} else {
		battery_state.batteryState = 0;
	}

	battery_state.legacyBatteryVoltage = battery_status.voltage_v * 10;
	return battery_state;
}

msp_rendor_battery_state_t construct_rendor_BATTERY_STATE(const battery_status_s &battery_status)
{
	// initialize result
	msp_rendor_battery_state_t battery_state = {0};

	battery_state.subCommand = MSP_DP_WRITE_STRING; // 3 write string. fixed
	// Position will be set by caller
	battery_state.screenYPosition = 0;
	battery_state.screenXPosition = 0;
	battery_state.iconAttrs = 0x00;

	float sigle_cell_v = 0.0f;

	// Check if battery is connected and has valid voltage
	if (!battery_status.connected || battery_status.voltage_v < 0.1f || battery_status.cell_count == 0) {
		// No battery connected or invalid data - show empty battery
		battery_state.iconIndex = 0x96; // Empty/Dead battery Icon
		battery_state.str[0] = '0';
		battery_state.str[1] = '.';
		battery_state.str[2] = '0';
		battery_state.str[3] = '\x06'; // Volt symbol
		battery_state.str_length = 4; // "0.0V"
		return battery_state;
	}

	sigle_cell_v = battery_status.voltage_v / battery_status.cell_count;

	if (sigle_cell_v > 4.0f) {
		battery_state.iconIndex = 0x91; // Full battery Icon

	} else if ((sigle_cell_v <= 4.0f) && (sigle_cell_v > 3.5f)) {
		battery_state.iconIndex = 0x93; // Half battery Icon

	} else if ((sigle_cell_v <= 3.5f) && (sigle_cell_v > 3.2f)) {
		battery_state.iconIndex = 0x95; // Empty battery Icon

	} else {
		battery_state.iconIndex = 0x96; // Dead battery Icon
	}

	// Display: battery_icon + voltage with volt symbol (0x06) - 4 chars exact, no null
	char temp[8];
	int len = snprintf(temp, sizeof(temp), "%.1f", (double)sigle_cell_v);
	if (len > 3) len = 3; // Max 3 chars before symbol
	memcpy(battery_state.str, temp, len);
	battery_state.str[len] = '\x06'; // Volt symbol at position len (max position 3)
	battery_state.str_length = len + 1; // digits + volt symbol
	return battery_state;
}


msp_raw_gps_t construct_RAW_GPS(const sensor_gps_s &vehicle_gps_position,
				const airspeed_validated_s &airspeed_validated)
{
	// initialize result
	msp_raw_gps_t raw_gps {0};

	if (vehicle_gps_position.fix_type >= 2) {
		raw_gps.lat = static_cast<int32_t>(vehicle_gps_position.latitude_deg * 1e7);
		raw_gps.lon = static_cast<int32_t>(vehicle_gps_position.longitude_deg * 1e7);
		raw_gps.alt = static_cast<int16_t>(vehicle_gps_position.altitude_msl_m * 100.0);

		float course = math::degrees(vehicle_gps_position.cog_rad);

		if (course < 0) {
			course += 360.0f;
		}

		raw_gps.groundCourse = course * 100.0f; // centidegrees

	} else {
		raw_gps.lat = 0;
		raw_gps.lon = 0;
		raw_gps.alt = 0;
		raw_gps.groundCourse = 0; // centidegrees
	}

	raw_gps.groundCourse = 0; // centidegrees

	if (vehicle_gps_position.fix_type == 0
	    || vehicle_gps_position.fix_type == 1) {
		raw_gps.fixType = MSP_GPS_NO_FIX;

	} else if (vehicle_gps_position.fix_type == 2) {
		raw_gps.fixType = MSP_GPS_FIX_2D;

	} else if (vehicle_gps_position.fix_type >= 3 && vehicle_gps_position.fix_type <= 5) {
		raw_gps.fixType = MSP_GPS_FIX_3D;

	} else {
		raw_gps.fixType = MSP_GPS_NO_FIX;
	}

	//raw_gps.hdop = vehicle_gps_position_struct.hdop
	raw_gps.numSat = vehicle_gps_position.satellites_used;

	if (airspeed_validated.airspeed_source >= airspeed_validated_s::SOURCE_GROUND_MINUS_WIND
	    && PX4_ISFINITE(airspeed_validated.indicated_airspeed_m_s)
	    && airspeed_validated.indicated_airspeed_m_s > 0.f) {
		raw_gps.groundSpeed = airspeed_validated.indicated_airspeed_m_s * 100;

	} else {
		raw_gps.groundSpeed = 0;
	}

	return raw_gps;
}

msp_rendor_latitude_t construct_rendor_GPS_LAT(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_latitude_t lat;

	// Position will be set by caller
	lat.screenYPosition = 0;
	lat.screenXPosition = 0;

	if (vehicle_gps_position.fix_type >= 2) {
		int len = snprintf(&lat.str[0], sizeof(lat.str), "%.6f", vehicle_gps_position.latitude_deg);
		lat.str_length = len;
	} else {
		int len = snprintf(&lat.str[0], sizeof(lat.str), "%.6f", 0.0);
		lat.str_length = len;
	}

	return lat;
}

msp_rendor_longitude_t construct_rendor_GPS_LON(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_longitude_t lon;

	// Position will be set by caller
	lon.screenYPosition = 0;
	lon.screenXPosition = 0;

	if (vehicle_gps_position.fix_type >= 2) {
		int len = snprintf(&lon.str[0], sizeof(lon.str), "%.6f", vehicle_gps_position.longitude_deg);
		lon.str_length = len;
	} else {
		int len = snprintf(&lon.str[0], sizeof(lon.str), "%.6f", -0.0);
		lon.str_length = len;
	}

	return lon;
}

msp_rendor_satellites_used_t construct_rendor_GPS_NUM(const sensor_gps_s &vehicle_gps_position)
{
	msp_rendor_satellites_used_t num;

	// Position will be set by caller
	num.screenYPosition = 0;
	num.screenXPosition = 0;

	memset(&num.str[0], 0, sizeof(num.str));
	int len = snprintf(&num.str[0], sizeof(num.str), "%d", vehicle_gps_position.satellites_used);
	num.str_length = len;

	return num;
}


msp_comp_gps_t construct_COMP_GPS(const home_position_s &home_position,
				  const vehicle_global_position_s &vehicle_global_position,
				  const bool heartbeat)
{
	// initialize result
	msp_comp_gps_t comp_gps {0};

	// Calculate distance and direction to home
	if (home_position.valid_hpos
	    && home_position.valid_lpos
	    && (hrt_elapsed_time(&vehicle_global_position.timestamp) < 1_s)) {

		float bearing_to_home = math::degrees(get_bearing_to_next_waypoint(vehicle_global_position.lat,
						      vehicle_global_position.lon,
						      home_position.lat, home_position.lon));

		if (bearing_to_home < 0) {
			bearing_to_home += 360.0f;
		}

		float distance_to_home = get_distance_to_next_waypoint(vehicle_global_position.lat,
					 vehicle_global_position.lon,
					 home_position.lat, home_position.lon);

		comp_gps.distanceToHome = (int16_t)distance_to_home; // meters
		comp_gps.directionToHome = bearing_to_home;

	} else {
		comp_gps.distanceToHome = 0; // meters
		comp_gps.directionToHome = 0;
	}

	comp_gps.heartbeat = heartbeat;
	return comp_gps;
}

msp_rendor_distanceToHome_t construct_rendor_distanceToHome(const home_position_s &home_position,
		const vehicle_global_position_s &vehicle_global_position)
{
	msp_rendor_distanceToHome_t distance;

	// Position will be set by caller
	distance.screenYPosition = 0;
	distance.screenXPosition = 0;

	memset(&distance.str[0], 0, sizeof(distance.str));

	if (home_position.valid_hpos
	    && home_position.valid_lpos
	    && (hrt_elapsed_time(&vehicle_global_position.timestamp) < 1_s)) {

		float distance_to_home = get_distance_to_next_waypoint(vehicle_global_position.lat,
					 vehicle_global_position.lon,
					 home_position.lat, home_position.lon);

		if (distance_to_home < 1000.0f) {
			// Display in meters: "999M" (3 digits + M symbol = 4 chars, pad with space)
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%d", (int)distance_to_home);
			int pos = 0;
			while (len < 3) { distance.str[pos++] = ' '; len++; } // Left pad with spaces
			memcpy(&distance.str[pos], temp, strlen(temp));
			pos += strlen(temp);
			distance.str[pos] = '\x0C'; // Meter symbol
			distance.str_length = pos + 1;
		} else if (distance_to_home < 10000.0f) {
			// Display 1.0 to 9.9 km with decimal: "1.1Km" (4 chars)
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%.1fK", (double)(distance_to_home / 1000.0f));
			memcpy(distance.str, temp, len);
			distance.str[len] = '\x0C'; // Meter symbol
			distance.str_length = len + 1;
		} else {
			// Display 10+ km without decimal: "10Km" (3-4 chars)
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%dK", (int)(distance_to_home / 1000.0f));
			memcpy(distance.str, temp, len);
			distance.str[len] = '\x0C'; // Meter symbol
			distance.str_length = len + 1;
		}

	} else {
		// No valid home position - 3 dashes + meter symbol (4 chars)
		distance.str[0] = '-';
		distance.str[1] = '-';
		distance.str[2] = '-';
		distance.str[3] = '\x0C'; // Meter symbol
		distance.str_length = 4;
	}

	return distance;
}

msp_rendor_pitch_t  construct_rendor_PITCH(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_rendor_pitch_t pit;

	// Position will be set by caller
	pit.screenYPosition = 0;
	pit.screenXPosition = 0;

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	double pitch_deg = (double)math::degrees(euler_attitude.theta());
	// attitude.roll = math::degrees(euler_attitude.phi()) * 10;

	memset(&pit.str[0], 0, sizeof(pit.str));
	int len = snprintf(&pit.str[0], sizeof(pit.str), "%.1f", pitch_deg);
	pit.str_length = len;

	return pit;
}

msp_rendor_roll_t  construct_rendor_ROLL(const vehicle_attitude_s &vehicle_attitude)
{
	// initialize results
	msp_rendor_roll_t roll;

	// Position will be set by caller
	roll.screenYPosition = 0;
	roll.screenXPosition = 0;

	// convert from quaternion to RPY
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	// double pitch = (double)math::degrees(euler_attitude.theta());
	double roll_deg = (double)math::degrees(euler_attitude.phi());

	memset(&roll.str[0], 0, sizeof(roll.str));
	int len = snprintf(&roll.str[0], sizeof(roll.str), "%.1f", roll_deg);
	roll.str_length = len;

	return roll;
}

void construct_rendor_HORIZON_SIDEBARS(const vehicle_attitude_s &vehicle_attitude,
	msp_rendor_horizon_sidebar_t &left, msp_rendor_horizon_sidebar_t &right)
{
	// Initialize both sidebars
	left.screenYPosition = 0;
	left.screenXPosition = 0;
	left.str[0] = '\0';
	left.str_length = 0; // Icon only, no text

	right.screenYPosition = 0;
	right.screenXPosition = 0;
	right.str[0] = '\0';
	right.str_length = 0; // Icon only, no text

	// Convert from quaternion to Euler angles
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	float roll_rad = euler_attitude.phi();

	// Calculate vertical offset based on roll angle
	// Grid is 20 rows (0-19), center at Y=10
	// Full roll range ±90° maps to ±10 rows
	// Positive roll (right wing down) -> left bar moves down, right bar moves up
	int8_t roll_offset = (int8_t)math::constrain((int)(roll_rad * 10.0f / 1.5708f), -10, 10);

	// Left sidebar moves opposite to roll (right wing down = left bar down)
	left.screenYPosition = 10 + roll_offset;

	// Right sidebar moves with roll (right wing down = right bar up)
	right.screenYPosition = 10 - roll_offset;
}


msp_altitude_t construct_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
				  const vehicle_local_position_s &vehicle_local_position)
{
	// initialize result
	msp_altitude_t altitude {0};

	if (vehicle_gps_position.fix_type >= 2) {
		altitude.estimatedActualPosition = static_cast<int32_t>(vehicle_gps_position.altitude_msl_m * 100.0);	// cm

	} else {
		altitude.estimatedActualPosition = 0;
	}

	if (vehicle_local_position.v_z_valid) {
		altitude.estimatedActualVelocity = -vehicle_local_position.vz * 100; //m/s to cm/s

	} else {
		altitude.estimatedActualVelocity = 0;
	}

	return altitude;
}

msp_rendor_altitude_t construct_Rendor_ALTITUDE(const sensor_gps_s &vehicle_gps_position,
		const vehicle_local_position_s &vehicle_local_position)
{
	msp_rendor_altitude_t altitude;

	// Position will be set by caller
	altitude.screenYPosition = 0;
	altitude.screenXPosition = 0;

	memset(&altitude.str[0], 0, sizeof(altitude.str));

	// Check if local position Z is valid
	if (vehicle_local_position.z_valid) {
		// Always use relative altitude from arming point (local Z position)
		float alt_m = vehicle_local_position.z * -1.0f;
		char temp[8];
		int len;

		// Show decimals only below 10m altitude
		if (alt_m < 10.0f && alt_m >= 0.0f) {
			len = snprintf(temp, sizeof(temp), "%.1f", (double)alt_m);
		} else {
			len = snprintf(temp, sizeof(temp), "%d", (int)alt_m);
		}

		if (len > 4) len = 4; // Max 4 digits before M symbol
		memcpy(altitude.str, temp, len);
		altitude.str[len] = '\x0C'; // Meter symbol
		altitude.str_length = len + 1; // digits + M symbol
	} else {
		// No valid altitude data - fill with dashes but keep meter symbol
		altitude.str[0] = '-';
		altitude.str[1] = '-';
		altitude.str[2] = '-';
		altitude.str[3] = '\x0C'; // Meter symbol
		altitude.str_length = 4; // "---M"
	}

	return altitude;
}

msp_esc_sensor_data_dji_t construct_ESC_SENSOR_DATA()
{
	// initialize result
	msp_esc_sensor_data_dji_t esc_sensor_data {0};

	esc_sensor_data.rpm = 0;
	esc_sensor_data.temperature = 50;

	return esc_sensor_data;
}

msp_rc_t construct_MSP_RC(const input_rc_s &input_rc)
{
	// initialize result
	msp_rc_t rc;

	rc.channelValue[0] = input_rc.values[0]; // roll
	rc.channelValue[1] = input_rc.values[1]; // pitch
	rc.channelValue[2] = input_rc.values[3]; // yaw
	rc.channelValue[3] = input_rc.values[2]; // Throttle
	return rc;
}

msp_status_t construct_MSP_STATUS(const vehicle_status_s &vehicle_status)
{
	// initialize result
	msp_status_t status{0};

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		status.flightModeFlags |= (1 << MSP_MODE_ARM);
	}

	return status;
}

// Additional rendor function implementations for missing OSD elements

msp_rendor_gps_speed_t construct_rendor_GPS_SPEED(const sensor_gps_s &vehicle_gps_position, int32_t speed_unit, int32_t speed_type)
{
	msp_rendor_gps_speed_t rendor_gps_speed{};

	// Position will be set by caller based on parameters
	rendor_gps_speed.screenYPosition = 0;
	rendor_gps_speed.screenXPosition = 0;

	// Check GPS fix validity
	if (vehicle_gps_position.fix_type >= 2 && (hrt_elapsed_time(&vehicle_gps_position.timestamp) < 1_s)) {
		// Calculate GPS speed manually (don't trust vel_m_s - inconsistent across GPS drivers)
		float speed_ms;
		if (speed_type == 1) {
			// 3D speed (total velocity including vertical)
			speed_ms = sqrtf(vehicle_gps_position.vel_n_m_s * vehicle_gps_position.vel_n_m_s +
			                 vehicle_gps_position.vel_e_m_s * vehicle_gps_position.vel_e_m_s +
			                 vehicle_gps_position.vel_d_m_s * vehicle_gps_position.vel_d_m_s);
		} else {
			// 2D horizontal speed (ground speed)
			speed_ms = sqrtf(vehicle_gps_position.vel_n_m_s * vehicle_gps_position.vel_n_m_s +
			                 vehicle_gps_position.vel_e_m_s * vehicle_gps_position.vel_e_m_s);
		}

		if (speed_unit == 1) {
			// km/h - format "123" + 0x9E icon (4 chars max)
			int speed_kmh = (int)(speed_ms * 3.6f);
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%d", speed_kmh);
			if (len > 3) len = 3;
			memcpy(rendor_gps_speed.str, temp, len);
			rendor_gps_speed.str[len] = '\x9E'; // km/h icon
			rendor_gps_speed.str_length = len + 1; // digits + icon
		} else {
			// m/s - format "123" + 0x9F icon (4 chars max)
			int speed_int = (int)speed_ms;
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%d", speed_int);
			if (len > 3) len = 3;
			memcpy(rendor_gps_speed.str, temp, len);
			rendor_gps_speed.str[len] = '\x9F'; // m/s icon
			rendor_gps_speed.str_length = len + 1; // digits + icon
		}
	} else {
		// No valid GPS - fill with dashes but keep unit symbol (4 chars)
		rendor_gps_speed.str[0] = '-';
		rendor_gps_speed.str[1] = '-';
		rendor_gps_speed.str[2] = '-';
		if (speed_unit == 1) {
			rendor_gps_speed.str[3] = '\x9E'; // km/h icon
		} else {
			rendor_gps_speed.str[3] = '\x9F'; // m/s icon
		}
		rendor_gps_speed.str_length = 4; // "---" + icon
	}

	return rendor_gps_speed;
}

msp_rendor_airspeed_t construct_rendor_AIRSPEED(const airspeed_validated_s &airspeed_validated, int32_t speed_unit, int32_t airspeed_source)
{
	msp_rendor_airspeed_t rendor_airspeed{};

	// Position will be set by caller based on parameters
	rendor_airspeed.screenYPosition = 0;
	rendor_airspeed.screenXPosition = 0;

	// Check if airspeed data is valid
	if (hrt_elapsed_time(&airspeed_validated.timestamp) < 1_s) {
		// Select airspeed source based on parameter
		float speed_ms = 0.0f;

		switch (airspeed_source) {
			case 0: // IAS (Indicated Airspeed)
				speed_ms = airspeed_validated.indicated_airspeed_m_s;
				break;
			case 1: // CAS (Calibrated Airspeed)
				speed_ms = airspeed_validated.calibrated_airspeed_m_s;
				break;
			case 2: // TAS (True Airspeed) - default
				speed_ms = airspeed_validated.true_airspeed_m_s;
				break;
			case 3: // CAS from ground-wind
				speed_ms = airspeed_validated.calibrated_ground_minus_wind_m_s;
				break;
			case 4: // TAS from ground-wind
				// v1.17 standardized AirspeedValidated and removed true_ground_minus_wind_m_s.
				// Fall back to the standard true airspeed (TAS).
				speed_ms = airspeed_validated.true_airspeed_m_s;
				break;
			default:
				speed_ms = airspeed_validated.true_airspeed_m_s;
				break;
		}

		if (speed_unit == 1) {
			// km/h - format "123" + 0x9E icon (4 chars max)
			int speed_kmh = (int)(speed_ms * 3.6f);
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%d", speed_kmh);
			if (len > 3) len = 3;
			memcpy(rendor_airspeed.str, temp, len);
			rendor_airspeed.str[len] = '\x9E'; // km/h icon
			rendor_airspeed.str_length = len + 1; // digits + icon
		} else {
			// m/s - format "123" + 0x9F icon (4 chars max)
			int speed_int = (int)speed_ms;
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%d", speed_int);
			if (len > 3) len = 3;
			memcpy(rendor_airspeed.str, temp, len);
			rendor_airspeed.str[len] = '\x9F'; // m/s icon
			rendor_airspeed.str_length = len + 1; // digits + icon
		}
	} else {
		// No valid airspeed data - fill with dashes but keep unit symbol (4 chars)
		rendor_airspeed.str[0] = '-';
		rendor_airspeed.str[1] = '-';
		rendor_airspeed.str[2] = '-';
		if (speed_unit == 1) {
			rendor_airspeed.str[3] = '\x9E'; // km/h icon
		} else {
			rendor_airspeed.str[3] = '\x9F'; // m/s icon
		}
		rendor_airspeed.str_length = 4; // "---" + icon
	}

	return rendor_airspeed;
}

msp_rendor_home_direction_t construct_rendor_HOME_DIR(const home_position_s &home_position,
		const vehicle_global_position_s &vehicle_global_position,
		const vehicle_attitude_s &vehicle_attitude)
{
	msp_rendor_home_direction_t rendor_home_dir{};

	// Position will be set by caller
	rendor_home_dir.screenYPosition = 0;
	rendor_home_dir.screenXPosition = 0;

	// Calculate distance to home to check if we're directly above
	float distance_to_home = get_distance_to_next_waypoint(vehicle_global_position.lat,
			 vehicle_global_position.lon,
			 home_position.lat, home_position.lon);

	// If within 2 meters horizontally, show "over home" symbol
	if (distance_to_home < 2.0f) {
		rendor_home_dir.iconIndex = 0x05; // SYM_OVER_HOME
		return rendor_home_dir;
	}

	// Get vehicle yaw (heading)
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	float yaw_deg = math::degrees(euler_attitude.psi());
	if (yaw_deg < 0) yaw_deg += 360.0f;

	// Calculate bearing from vehicle to home (absolute geographic direction)
	double lat1 = vehicle_global_position.lat * 1e-7;
	double lon1 = vehicle_global_position.lon * 1e-7;
	double lat2 = home_position.lat * 1e-7;
	double lon2 = home_position.lon * 1e-7;

	double dLon = (lon2 - lon1) * M_PI / 180.0;
	lat1 = lat1 * M_PI / 180.0;
	lat2 = lat2 * M_PI / 180.0;

	double y = sin(dLon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
	double bearing = atan2(y, x) * 180.0 / M_PI;
	if (bearing < 0) bearing += 360.0;

	// Calculate RELATIVE bearing (where is home relative to our heading)
	float relative_bearing = (float)bearing - yaw_deg;

	// Normalize to -180 to +180
	while (relative_bearing > 180.0f) relative_bearing -= 360.0f;
	while (relative_bearing < -180.0f) relative_bearing += 360.0f;

	// Betaflight font arrows rotate counter-clockwise with:
	// 0x60 (idx 0) = 180° down/behind (home behind drone)
	// 0x64 (idx 4) = 90° right (home to the right)
	// 0x68 (idx 8) = 0° up/ahead (home ahead)
	// 0x6C (idx 12) = 270° left (home to the left)
	// Formula: arrow_angle = 180° - relative_bearing
	float arrow_angle = 180.0f - relative_bearing;

	// Normalize arrow_angle to 0-360
	while (arrow_angle < 0.0f) arrow_angle += 360.0f;
	while (arrow_angle >= 360.0f) arrow_angle -= 360.0f;

	// Map to 16 arrow directions (each covers 22.5°)
	int arrow_idx = (int)((arrow_angle + 11.25f) / 22.5f) % 16;

	// Arrow symbols: 0x60-0x6F (16 directions)
	rendor_home_dir.iconIndex = 0x60 + arrow_idx;

	return rendor_home_dir;
}

msp_rendor_power_t construct_rendor_POWER(const battery_status_s &battery_status)
{
	msp_rendor_power_t rendor_power{};

	// Position will be set by caller
	rendor_power.screenYPosition = 0;
	rendor_power.screenXPosition = 0;

	if (battery_status.connected && battery_status.voltage_v > 0.0f) {
		// Power = Voltage * Current
		float voltage = battery_status.voltage_v;
		float current = battery_status.current_a;
		float power_watts = voltage * current;

		if (power_watts < 1000.0f) {
			// Display in watts: "999W" (4 chars max)
			char temp[6];
			int len = snprintf(temp, sizeof(temp), "%d", (int)power_watts);
			if (len > 5) len = 5;
			memcpy(rendor_power.str, temp, len);
			rendor_power.str[len] = '\x57'; // W symbol
			rendor_power.str_length = len + 1;
		} else {
			// Display in kilowatts: "1.2KW" (5 chars max)
			char temp[8];
			int len = snprintf(temp, sizeof(temp), "%.1fK", (double)(power_watts / 1000.0f));
			if (len > 5) len = 5;
			memcpy(rendor_power.str, temp, len);
			rendor_power.str[len] = '\x57'; // W symbol
			rendor_power.str_length = len + 1;
		}
	} else {
		// No valid battery data - keep W symbol (5 chars)
		rendor_power.str[0] = '-';
		rendor_power.str[1] = '-';
		rendor_power.str[2] = '-';
		rendor_power.str[3] = '-';
		rendor_power.str[4] = '\x57'; // W symbol
		rendor_power.str[5] = ' '; // Fill last char
		rendor_power.str_length = 6;
	}

	return rendor_power;
}

msp_rendor_vspeed_t construct_rendor_VSPEED(const vehicle_local_position_s &vehicle_local_position)
{
	msp_rendor_vspeed_t rendor_vspeed{};

	// Position will be set by caller
	rendor_vspeed.screenYPosition = 0;
	rendor_vspeed.screenXPosition = 0;

	// Check if vz is valid
	if (vehicle_local_position.v_z_valid && (hrt_elapsed_time(&vehicle_local_position.timestamp) < 1_s)) {
		// Vertical speed (negative of vz since NED frame)
		float vspeed_ms = -vehicle_local_position.vz;

		// Icon selection: near-zero = horizontal bar, otherwise up/down arrow
		if (fabsf(vspeed_ms) < 0.1f) {
			rendor_vspeed.iconIndex = 0x84; // SYM_AH_BAR9_4 (horizontal bar for near-zero)
		} else {
			rendor_vspeed.iconIndex = (vspeed_ms >= 0) ? 0x75 : 0x76; // UP=0x75, DOWN=0x76
		}

		// Display in m/s with one decimal + m/s symbol (0x9F)
		char temp[8];
		int len = snprintf(temp, sizeof(temp), "%.1f", (double)fabsf(vspeed_ms));
		if (len > 5) len = 5;
		memcpy(rendor_vspeed.str, temp, len);
		rendor_vspeed.str[len] = '\x9F'; // SYM_MS (m/s symbol)
		rendor_vspeed.str_length = len + 1; // digits + symbol
	} else {
		// No valid vertical speed data
		rendor_vspeed.iconIndex = 0x84; // Horizontal bar for no data
		memcpy(rendor_vspeed.str, "---\x9F", 4); // "---" + m/s symbol
		rendor_vspeed.str_length = 4;
	}

	return rendor_vspeed;
}

msp_rendor_flight_mode_t construct_rendor_FLIGHT_MODE(const vehicle_status_s &vehicle_status, bool use_short_format)
{
	msp_rendor_flight_mode_t rendor_mode{};

	// Position will be set by caller
	rendor_mode.screenYPosition = 0;
	rendor_mode.screenXPosition = 0;

	// Map PX4 nav_state to readable flight mode names
	const char* mode_name = "UNKNOWN";
	const char* mode_short = "UNK";

	switch (vehicle_status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			mode_name = "MANUAL";
			mode_short = "MAN";
			break;
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			mode_name = "ALT HOLD";
			mode_short = "ALT";
			break;
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			mode_name = "POS HOLD";
			mode_short = "POS";
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			mode_name = "MISSION";
			mode_short = "MIS";
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			mode_name = "LOITER";
			mode_short = "LOI";
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
			mode_name = "RTL";
			mode_short = "RTL";
			break;
		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			mode_name = "ACRO";
			mode_short = "ACR";
			break;
		case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			mode_name = "OFFBOARD";
			mode_short = "OFF";
			break;
		case vehicle_status_s::NAVIGATION_STATE_STAB:
			mode_name = "STABILIZE";
			mode_short = "STA";
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			mode_name = "TAKEOFF";
			mode_short = "TKO";
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
			mode_name = "LAND";
			mode_short = "LND";
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
			mode_name = "FOLLOW";
			mode_short = "FOL";
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
			mode_name = "PRECLAND";
			mode_short = "PRC";
			break;
		default:
			mode_name = "UNKNOWN";
			mode_short = "UNK";
			break;
	}

	// Copy mode string without null terminator (max 8 chars)
	const char* mode_str = use_short_format ? mode_short : mode_name;
	int len = strlen(mode_str);
	if (len > 8) len = 8;
	memcpy(rendor_mode.str, mode_str, len);
	rendor_mode.str_length = len; // Dynamic length: "MAN"=3, "MANUAL"=6, etc.

	return rendor_mode;
}

msp_rendor_crosshairs_t construct_rendor_CROSSHAIRS()
{
	msp_rendor_crosshairs_t rendor_cross{};

	// Position will be set by caller
	rendor_cross.screenYPosition = 0;
	rendor_cross.screenXPosition = 0;

	// Crosshair with 3 symbols (no null terminator)
	rendor_cross.str[0] = '\x72'; // SYM_AH_CENTER_LINE (left)
	rendor_cross.str[1] = '\x73'; // SYM_AH_CENTER
	rendor_cross.str[2] = '\x74'; // SYM_AH_CENTER_LINE_RIGHT
	rendor_cross.str_length = 3;

	return rendor_cross;
}

void construct_rendor_ARTIFICIAL_HORIZON(const vehicle_attitude_s &vehicle_attitude, int crosshair_y, int crosshair_x, int width,
	float pitch_offset, msp_rendor_horizon_bar_t &horizon_bar, msp_rendor_horizon_cursor_t &cursor_left, msp_rendor_horizon_cursor_t &cursor_right)
{
	// Calculate pitch angle in degrees
	matrix::Eulerf euler(matrix::Quatf(vehicle_attitude.q));
	float pitch_deg = math::degrees(euler.theta());

	// Apply pitch offset to compensate for PX4 mounting misalignment
	pitch_deg += pitch_offset;

	// FOV calculation: Adjust deg_per_row to match your camera/goggle setup
	// Lower value = graduations more spaced out vertically
	const float deg_per_row = 2.5f;
	const float deg_per_bar_level = deg_per_row / 9.0f;

	// Calculate total offset in bar levels (positive pitch = horizon down)
	float total_bar_offset = pitch_deg / deg_per_bar_level;

	// Split into Y offset (integer rows) and bar level (0-8)
	int y_offset = (int)(total_bar_offset / 9.0f);
	int bar_level = (int)(total_bar_offset) % 9;
	if (bar_level < 0) {
		bar_level += 9;
		y_offset -= 1;
	}
	if (bar_level > 8) bar_level = 8;
	if (bar_level < 0) bar_level = 0;

	// Calculate horizon Y position
	int horizon_y = crosshair_y + y_offset;

	// Clamp to screen bounds and adjust bar level at limits
	if (horizon_y < 0) {
		horizon_y = 0;
		// At top limit (Y=0): bar should be at top (level 0)
		if (y_offset < 0) bar_level = 0;
	}
	if (horizon_y > 19) {
		horizon_y = 19;
		// At bottom limit (Y=19): bar should be at bottom (level 8)
		if (y_offset > 0) bar_level = 8;
	}

	// Build horizon bar (variable width, centered around crosshair)
	horizon_bar.screenYPosition = horizon_y;

	// Clamp width to max buffer size
	if (width > 45) width = 45;
	if (width < 3) width = 3;

	// Center horizon bar around crosshair X position
	horizon_bar.screenXPosition = crosshair_x - width / 2;

	// Select bar symbol (0x80-0x88)
	char bar_symbol = (char)(0x80 + bar_level);
	for (int i = 0; i < width; i++) {
		horizon_bar.str[i] = bar_symbol;
	}
	horizon_bar.str_length = width;

	// Cursors stay aligned with crosshair Y (don't move vertically)
	// Position them just outside the horizon bar edges
	int bar_start_x = crosshair_x - width / 2;
	int bar_end_x = bar_start_x + width - 1;

	cursor_left.screenYPosition = crosshair_y;
	cursor_left.screenXPosition = bar_start_x - 1; // Left of horizon bar
	cursor_left.iconIndex = 0x03; // SYM_AH_LEFT / SYM_CURSOR

	cursor_right.screenYPosition = crosshair_y;
	cursor_right.screenXPosition = bar_end_x + 1; // Right of horizon bar
	cursor_right.iconIndex = 0x02; // SYM_AH_RIGHT
}

int construct_rendor_PITCH_LADDER(const vehicle_attitude_s &vehicle_attitude,
	int crosshair_y, int crosshair_x, int ladder_width, int ladder_side, int ladder_step,
	float pitch_offset, msp_rendor_pitch_ladder_line_t lines[], int max_lines)
{
	// Calculate current pitch angle in degrees
	matrix::Eulerf euler(matrix::Quatf(vehicle_attitude.q));
	float pitch_deg = math::degrees(euler.theta());

	// Apply pitch offset to compensate for PX4 mounting misalignment
	// This shifts the ladder graduations to align with actual horizon
	pitch_deg += pitch_offset;

	// Screen parameters
	// FOV: ~80° vertical / 20 rows = 4°/row
	// 9 bar levels per row (0x80-0x88) = 4°/9 = 0.444° per bar level
	const float deg_per_row = 4.0f;
	const float deg_per_bar_level = deg_per_row / 9.0f; // 0.444°
	const int pitch_increment = ladder_step; // Use configured graduation step

	// Screen bounds
	const int screen_min_y = 0;
	const int screen_max_y = 21;

	int line_count = 0;

	// 1. MOVING PITCH GRADUATIONS with numbers + variable bars (draw FIRST)
	// Each graduation moves in Y and varies bar level for smooth motion
	// Graduation spacing (pitch_increment) is configured via OSD_LADDER_STEP parameter
	for (int angle = -90; angle <= 90 && line_count < max_lines; angle += pitch_increment) {
		float angle_offset_deg = pitch_deg - angle;

		// Convert to total bar levels offset from graduation position
		// When angle_offset=0 (pitch=angle), we want bar_level=4 (middle bar)
		float total_bar_offset = angle_offset_deg / deg_per_bar_level;

		// Add 4 to shift the scale so bar_level=4 when offset=0
		int shifted_offset = (int)floorf(total_bar_offset + 4.0f + 0.5f);

		// Split into Y offset and bar level
		int y_offset = shifted_offset / 9;
		int bar_level = shifted_offset % 9;

		// Handle negative modulo
		if (bar_level < 0) {
			bar_level += 9;
			y_offset -= 1;
		}

		// Adjust y_offset to account for the initial +4 shift
		y_offset -= 0;  // Already accounted for in the shift

		// Calculate final Y position
		int y_pos = crosshair_y + y_offset;

		// Only draw if visible on screen
		if (y_pos < screen_min_y || y_pos > screen_max_y) {
			continue;
		}

		// Build the graduation line with segments on sides
		msp_rendor_pitch_ladder_line_t &line = lines[line_count];
		line.screenYPosition = y_pos;
		line.iconAttrs = 0x00;

		// Bar symbol varies based on position within the Y cell
		char bar_symbol = (char)(0x80 + bar_level);

		// Format the angle number (ensure consistent width)
		char num_str[8];
		int num_len;

		if (angle >= 0) {
			num_len = snprintf(num_str, sizeof(num_str), "%d", angle);
		} else {
			// For negative numbers, use standard minus
			num_len = snprintf(num_str, sizeof(num_str), "%d", angle);
		}

		// Clamp to reasonable size
		if (num_len > 3) num_len = 3;

		int str_idx = 0;
		const int bar_segment = 2; // 2 bars on each side

		// Gap size = horizon width (ladder_width)
		// Graduations are placed based on ladder_side parameter
		int gap_size = ladder_width;

		// Build graduation based on selected side(s)
		// ladder_side: 1=left, 2=right, 3=both

		if (ladder_side == 1 || ladder_side == 3) {
			// LEFT SIDE: num + bars positioned before left edge of horizon
			line.screenXPosition = crosshair_x - ladder_width / 2 - bar_segment - num_len;

			// Left number
			for (int i = 0; i < num_len && str_idx < 51; i++) {
				line.str[str_idx++] = num_str[i];
			}

			// Left bars
			for (int i = 0; i < bar_segment && str_idx < 51; i++) {
				line.str[str_idx++] = bar_symbol;
			}
		} else if (ladder_side == 2) {
			// RIGHT SIDE ONLY: position at right edge (+1 offset for alignment with 'both' mode)
			line.screenXPosition = crosshair_x + ladder_width / 2 + 1;
		}

		// Middle gap (for both sides mode)
		if (ladder_side == 3) {
			for (int i = 0; i < gap_size && str_idx < 51; i++) {
				line.str[str_idx++] = ' ';
			}
		}

		// RIGHT SIDE
		if (ladder_side == 2 || ladder_side == 3) {
			// Right bars
			for (int i = 0; i < bar_segment && str_idx < 51; i++) {
				line.str[str_idx++] = bar_symbol;
			}

			// Right number
			for (int i = 0; i < num_len && str_idx < 51; i++) {
				line.str[str_idx++] = num_str[i];
			}
		}

		line.str_length = str_idx;
		line_count++;
	}

	// 2. DRAW 0° GRADUATION - Always include it even if not in ladder_step sequence
	// This ensures the zero reference is always visible
	if (line_count < max_lines) {
		int angle = 0;
		float angle_offset_deg = pitch_deg - angle;

		// Convert to total bar levels offset from graduation position
		// When angle_offset=0 (pitch=0), we want bar_level=4 (middle bar)
		float total_bar_offset = angle_offset_deg / deg_per_bar_level;

		// Add 4 to shift the scale so bar_level=4 when offset=0
		int shifted_offset = (int)floorf(total_bar_offset + 4.0f + 0.5f);

		// Split into Y offset and bar level
		int y_offset = shifted_offset / 9;
		int bar_level = shifted_offset % 9;

		// Handle negative modulo
		if (bar_level < 0) {
			bar_level += 9;
			y_offset -= 1;
		}

		// Calculate final Y position
		int y_pos = crosshair_y + y_offset;

		// Only draw if visible on screen
		if (y_pos >= screen_min_y && y_pos <= screen_max_y) {
			// Build the 0° graduation line
			msp_rendor_pitch_ladder_line_t &line = lines[line_count];
			line.screenYPosition = y_pos;
			line.iconAttrs = 0x00;

			// Bar symbol varies based on position within the Y cell
			char bar_symbol = (char)(0x80 + bar_level);

		int str_idx = 0;
		const int bar_segment = 2; // 2 bars on each side
		int gap_size = ladder_width;

		// Build graduation based on selected side(s)
		if (ladder_side == 1 || ladder_side == 3) {
			// LEFT SIDE: "0" + bars positioned before left edge of horizon
			line.screenXPosition = crosshair_x - ladder_width / 2 - bar_segment - 1;

			// Left number "0"
			line.str[str_idx++] = '0';

			// Left bars
			for (int i = 0; i < bar_segment && str_idx < 51; i++) {
				line.str[str_idx++] = bar_symbol;
			}
		} else if (ladder_side == 2) {
			// RIGHT SIDE ONLY: position at right edge (+1 offset for alignment with 'both' mode)
			line.screenXPosition = crosshair_x + ladder_width / 2 + 1;
		}

		// Middle gap (for both sides mode)
		if (ladder_side == 3) {
			for (int i = 0; i < gap_size && str_idx < 51; i++) {
				line.str[str_idx++] = ' ';
			}
		}

		// RIGHT SIDE
		if (ladder_side == 2 || ladder_side == 3) {
			// Right bars
			for (int i = 0; i < bar_segment && str_idx < 51; i++) {
				line.str[str_idx++] = bar_symbol;
			}

			// Right number "0"
			line.str[str_idx++] = '0';
		}

		line.str_length = str_idx;
		line_count++;
	}
}

	// 3. FIXED HORIZON LINE - Draw LAST so it appears on top (no number, just bar)
	// Always at crosshair_y with bar level 4 (middle)
	if (line_count < max_lines) {
		msp_rendor_pitch_ladder_line_t &horizon_line = lines[line_count];
		horizon_line.screenYPosition = crosshair_y;
		horizon_line.screenXPosition = crosshair_x - ladder_width / 2;
		horizon_line.iconAttrs = 0x00;

		// Bar level 4 = middle bar (0x84) - FIXED horizon reference
		char bar_symbol = 0x84;
		int str_idx = 0;
		for (int i = 0; i < ladder_width && str_idx < 51; i++) {
			horizon_line.str[str_idx++] = bar_symbol;
		}
		horizon_line.str_length = str_idx;
		line_count++;
	}

	return line_count;
}

void construct_rendor_HEADING(const vehicle_attitude_s &vehicle_attitude, const home_position_s &home_position, const vehicle_global_position_s &vehicle_global_position, msp_rendor_heading_t &row1, msp_rendor_heading_t &row2)
{
	// Initialize both rows
	row1.screenYPosition = 0;
	row1.screenXPosition = 0;
	row2.screenYPosition = 0;
	row2.screenXPosition = 0;

	const auto now = hrt_absolute_time();

	// Check if attitude data is recent
	if (vehicle_attitude.timestamp < (now - 1_s)) {
		memcpy(row1.str, "   HEADING?    ", 15);
		memcpy(row2.str, "   -------    ", 14);
		return;
	}

	// Convert quaternion to Euler angles and normalize to 0-360
	matrix::Eulerf euler_attitude(matrix::Quatf(vehicle_attitude.q));
	float yaw_deg = math::degrees(euler_attitude.psi());
	if (yaw_deg < 0) yaw_deg += 360.0f;

	// Calculate home direction
	float home_direction = -1.0f; // -1 means no home
	float home_distance = 0.0f;

	if (home_position.valid_lpos && vehicle_global_position.timestamp > 0) {
		// Calculate bearing to home
		double lat1 = vehicle_global_position.lat;
		double lon1 = vehicle_global_position.lon;
		double lat2 = home_position.lat;
		double lon2 = home_position.lon;

		double lat_diff = lat2 - lat1;
		double lon_diff = lon2 - lon1;

		// Distance in meters (approximate)
		float lat_dist = (float)(lat_diff * 111320.0);
		float lon_dist = (float)(lon_diff * 111320.0 * cos(math::radians(lat1)));
		home_distance = sqrtf(lat_dist * lat_dist + lon_dist * lon_dist);

		// Bearing calculation
		double y = sin(math::radians(lon_diff)) * cos(math::radians(lat2));
		double x = cos(math::radians(lat1)) * sin(math::radians(lat2)) -
		           sin(math::radians(lat1)) * cos(math::radians(lat2)) * cos(math::radians(lon_diff));
		float bearing = math::degrees(atan2(y, x));
		if (bearing < 0) bearing += 360.0f;

		home_direction = bearing;
	}

	// Create heading bars - 19 characters wide each
	char bar_row[20] =       "                   "; // Top row: Bar with arrow
	char cardinals_row[20] = "                   "; // Bottom row: Cardinal letters

	// Cardinal directions symbols: N=0x18, S=0x19, E=0x1A, W=0x1B
	const uint8_t cardinal_symbols[] = {0x18, 0, 0x1A, 0, 0x19, 0, 0x1B, 0}; // N, NE(skip), E, SE(skip), S, SW(skip), W, NW(skip)

	// Each cardinal is 45° apart, bar shows ±90° (180° total view)
	for (int i = 0; i < 8; i++) {
		// Only show main cardinals (N, E, S, W) - skip NE, SE, SW, NW
		if (i % 2 != 0) continue;

		float cardinal_angle = i * 45.0f;

		// Calculate relative position: where is the cardinal relative to our heading?
		float diff = cardinal_angle - yaw_deg;

		// Normalize difference to -180 to +180
		while (diff > 180.0f) diff -= 360.0f;
		while (diff < -180.0f) diff += 360.0f;

		// Check if this cardinal is visible (within ±90°)
		if (diff >= -90.0f && diff <= 90.0f) {
			// Map to bar position (0-18, with 9 being center)
			int bar_pos = (int)(9.5f + (diff / 90.0f) * 9.5f);

			if (bar_pos >= 0 && bar_pos < 19) {
				// Place cardinal symbol on bottom row
				cardinals_row[bar_pos] = cardinal_symbols[i];

				// Place long line on cardinals
				bar_row[bar_pos] = '\x1D'; // SYM_HEADING_LINE (long)
			}
		}
	}

	// Add HOME marker (H) if valid and not too close (> 5m)
	if (home_direction >= 0.0f && home_distance > 5.0f) {
		float home_diff = home_direction - yaw_deg;

		// Normalize
		while (home_diff > 180.0f) home_diff -= 360.0f;
		while (home_diff < -180.0f) home_diff += 360.0f;

		// Check if home is visible (within ±90°)
		if (home_diff >= -90.0f && home_diff <= 90.0f) {
			int home_pos = (int)(9.5f + (home_diff / 90.0f) * 9.5f);

			if (home_pos >= 0 && home_pos < 19) {
				// Place 'H' on bottom row
				cardinals_row[home_pos] = 'H';
			}
		}
	}

	// Fill bar row with short lines between cardinals
	for (int i = 0; i < 19; i++) {
		if (bar_row[i] == ' ') {
			bar_row[i] = '\x1C'; // SYM_HEADING_DIVIDED_LINE (short)
		}
	}

	// Add center indicator (down arrow) only on top row at position 9
	bar_row[9] = '\x60';       // SYM_ARROW_SOUTH (down arrow)

	memcpy(row1.str, bar_row, 19);
	row1.str_length = 19; // Fixed: heading bar always 19 chars (visual continuity)
	memcpy(row2.str, cardinals_row, 19);
	row2.str_length = 19; // Fixed: cardinal row always 19 chars (visual continuity)
}

msp_rendor_arming_t construct_rendor_ARMING(const vehicle_status_s &vehicle_status)
{
	msp_rendor_arming_t arming{};

	// Position will be set by caller
	arming.screenYPosition = 0;
	arming.screenXPosition = 0;

	if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		memcpy(arming.str, "ARMED", 5);
		arming.str_length = 5; // "ARMED"=5 bytes
		arming.iconAttrs = 0x03; // Red - danger
	} else {
		memcpy(arming.str, "DISARMED", 8);
		arming.str_length = 8; // "DISARMED"=8 bytes
		arming.iconAttrs = 0x01; // Green - safe
	}

	return arming;
}

msp_rendor_warning_t construct_rendor_WARNING(const log_message_s &log_message, const int log_level)
{
	msp_rendor_warning_t warning{};

	// Position will be set by caller
	warning.screenYPosition = 0;
	warning.screenXPosition = 0;

	const auto now = hrt_absolute_time();
	static uint64_t last_warning_stamp = 0;
	static char current_message[MSG_BUFFER_SIZE] = "";
	static uint16_t scroll_index = 0;
	static uint64_t last_scroll_time = 0;

	// Update warning text with prefix based on severity
	if (log_message.severity <= log_level && log_message.text[0] != '\0') {
		// Ensure log_message.text is null-terminated (it's 127 chars, might not be terminated)
		char safe_text[128];
		memcpy(safe_text, log_message.text, 127);
		safe_text[127] = '\0';

		// Add prefix based on severity (Linux kernel log levels 0-7)
		const char *prefix;
		switch (log_message.severity) {
		case 0: prefix = "EMERG: "; break;
		case 1: prefix = "ALERT: "; break;
		case 2: prefix = "CRIT: "; break;
		case 3: prefix = "ERROR: "; break;
		case 4: prefix = "WARN: "; break;
		case 5: prefix = "NOTICE: "; break;
		case 6: prefix = "INFO: "; break;
		case 7: prefix = "DEBUG: "; break;
		default: prefix = "LOG: "; break;
		}

		// Build full message with prefix
		snprintf(current_message, sizeof(current_message), "%s%s", prefix, safe_text);
		last_warning_stamp = now;
		scroll_index = 0; // Reset scroll on new message

	} else if (now - last_warning_stamp > 30_s) {
		// Clear warning after timeout
		current_message[0] = '\0';
		scroll_index = 0;
		last_warning_stamp = now;
	}

	// Simple scrolling logic
	int msg_len = strlen(current_message);
	const int max_display = (int)sizeof(warning.str);

	if (msg_len <= max_display) {
		// Message fits, no scrolling needed
		memcpy(warning.str, current_message, msg_len);
		warning.str_length = msg_len;
	} else {
		// Message too long, scroll it
		const uint64_t scroll_period = 125000; // 125ms between scrolls
		const uint64_t dwell_time = 500000;    // 500ms pause at start

		if (scroll_index == 0 && now - last_scroll_time >= dwell_time) {
			// Dwell at start, then start scrolling
			last_scroll_time = now;
			scroll_index++;
		} else if (scroll_index > 0 && now - last_scroll_time >= scroll_period) {
			// Continue scrolling
			last_scroll_time = now;
			scroll_index++;
			if (scroll_index > msg_len - max_display) {
				scroll_index = 0; // Loop back to start
			}
		}

		// Copy visible portion
		memcpy(warning.str, current_message + scroll_index, max_display);
		warning.str_length = max_display;
	}

	// Set color based on severity
	if (log_message.severity <= 3) {
		warning.iconAttrs = 0x43; // Red + blink for errors
	} else if (log_message.severity == 4) {
		warning.iconAttrs = 0x42; // Orange + blink for warnings
	} else {
		warning.iconAttrs = 0x00; // White for logs
	}

	return warning;
}

msp_rendor_esc_temp_t construct_rendor_ESC_TEMP(const esc_status_s &esc_status)
{
	msp_rendor_esc_temp_t esc_temp{};

	// Position will be set by caller
	esc_temp.screenYPosition = 0;
	esc_temp.screenXPosition = 0;

	// Find maximum ESC temperature
	float max_temp = -273.15f; // Absolute zero as invalid default
	bool valid_temp = false;

	for (int i = 0; i < esc_status.esc_count && i < 8; i++) {
		if (esc_status.esc[i].esc_temperature > max_temp) {
			max_temp = esc_status.esc[i].esc_temperature;
			valid_temp = true;
		}
	}

	if (valid_temp && max_temp > -100.0f) {
		int temp_celsius = (int)max_temp;
		char temp[12];
		int len = snprintf(temp, sizeof(temp), "ESC:%d", temp_celsius);
		if (len > 11) len = 11; // Max 11 chars before °C symbol (buffer size 12)
		memcpy(esc_temp.str, temp, len);
		esc_temp.str[len] = '\x0E'; // SYM_C (°C symbol)
		esc_temp.str_length = len + 1; // digits + °C symbol

		// Change color based on temperature
		if (temp_celsius > 80) {
			esc_temp.iconAttrs = 0x03; // Red if hot (>80°C)
		} else if (temp_celsius > 60) {
			esc_temp.iconAttrs = 0x00; // White if warm (60-80°C)
		} else {
			esc_temp.iconAttrs = 0x00; // White if normal
		}
	} else {
		// No ESC data - keep Celsius symbol
		int len = snprintf(esc_temp.str, sizeof(esc_temp.str), "ESC:---\x0E");
		esc_temp.str_length = len;
		esc_temp.iconAttrs = 0x00;
	}

	return esc_temp;
}

msp_rendor_esc_rpm_t construct_rendor_ESC_RPM(const esc_status_s &esc_status)
{
	msp_rendor_esc_rpm_t esc_rpm{};

	// Position will be set by caller
	esc_rpm.screenYPosition = 0;
	esc_rpm.screenXPosition = 0;

	// Initialize buffer with zeros
	memset(esc_rpm.str, 0, sizeof(esc_rpm.str));

	// Find maximum ESC RPM
	int32_t max_rpm = 0;
	bool valid_rpm = false;

	for (int i = 0; i < esc_status.esc_count && i < 8; i++) {
		if (esc_status.esc[i].esc_rpm > max_rpm) {
			max_rpm = esc_status.esc[i].esc_rpm;
		}
		valid_rpm = true; // Mark as valid if we have at least one ESC
	}

	if (valid_rpm) {
		// Format: "RPM:" prefix + digits (show 0 if motors stopped, --- only if no ESC data)
		int len = snprintf(esc_rpm.str, sizeof(esc_rpm.str), "RPM:%u", (unsigned int)max_rpm);
		esc_rpm.str_length = len;
		esc_rpm.iconAttrs = 0x00; // White
	} else {
		// No ESC data: "RPM:-----"
		int len = snprintf(esc_rpm.str, sizeof(esc_rpm.str), "RPM:-----");
		esc_rpm.str_length = len;
		esc_rpm.iconAttrs = 0x00;
	}

	return esc_rpm;
}

msp_rendor_esc_amp_t construct_rendor_ESC_AMP(const esc_status_s &esc_status)
{
	msp_rendor_esc_amp_t esc_amp{};

	// Position will be set by caller
	esc_amp.screenYPosition = 0;
	esc_amp.screenXPosition = 0;

	// Initialize buffer with zeros
	memset(esc_amp.str, 0, sizeof(esc_amp.str));

	// Find maximum ESC current
	float max_current = 0.0f;
	bool valid_current = false;

	for (int i = 0; i < esc_status.esc_count && i < 8; i++) {
		if (esc_status.esc[i].esc_current > max_current) {
			max_current = esc_status.esc[i].esc_current;
		}
		if (esc_status.esc[i].esc_current >= 0.0f) { // Valid if >= 0
			valid_current = true;
		}
	}

	if (valid_current) {
		// Format current with one decimal place
		if (max_current < 100.0f) {
			char temp[8];
			int len = snprintf(temp, sizeof(temp), "%.1f", (double)max_current);
			if (len > 4) len = 4; // Reserve space for symbol
			memcpy(esc_amp.str, temp, len);
			esc_amp.str[len] = '\x9A'; // Ampere symbol
			esc_amp.str_length = len + 1;
		} else {
			// No decimal for values >= 100A
			char temp[8];
			int len = snprintf(temp, sizeof(temp), "%.0f", (double)max_current);
			if (len > 4) len = 4;
			memcpy(esc_amp.str, temp, len);
			esc_amp.str[len] = '\x9A'; // Ampere symbol
			esc_amp.str_length = len + 1;
		}

		// Color coding: red if current is very high (>50A)
		if (max_current > 50.0f) {
			esc_amp.iconAttrs = 0x03; // Red
		} else {
			esc_amp.iconAttrs = 0x00; // White
		}
	} else {
		// No ESC current data: "----A"
		memcpy(esc_amp.str, "----\x9A", 5);
		esc_amp.str_length = 5;
		esc_amp.iconAttrs = 0x00;
	}

	return esc_amp;
}

msp_rendor_current_t construct_rendor_CURRENT(const battery_status_s &battery_status)
{
	msp_rendor_current_t current{};

	// Position will be set by caller
	current.screenYPosition = 0;
	current.screenXPosition = 0;

	if (battery_status.connected && battery_status.current_a > -1000.0f) {
		float current_amps = fabsf(battery_status.current_a);

		if (current_amps < 100.0f) {
			char temp[8];
			int len = snprintf(temp, sizeof(temp), "%.1f", (double)current_amps);
			if (len > 5) len = 5;
			memcpy(current.str, temp, len);
			current.str[len] = '\x9A'; // Ampere symbol
			current.str_length = len + 1;
		} else {
			char temp[8];
			int len = snprintf(temp, sizeof(temp), "%.0f", (double)current_amps);
			if (len > 5) len = 5;
			memcpy(current.str, temp, len);
			current.str[len] = '\x9A'; // Ampere symbol
			current.str_length = len + 1;
		}

		// Red if current is very high (>50A)
		if (current_amps > 50.0f) {
			current.iconAttrs = 0x03;
		} else {
			current.iconAttrs = 0x00;
		}
	} else {
		current.str[0] = '-';
		current.str[1] = '-';
		current.str[2] = '-';
		current.str[3] = '-';
		current.str[4] = '\x9A'; // Ampere symbol
		current.str[5] = ' '; // Fill
		current.str_length = 6;
		current.iconAttrs = 0x00;
	}

	return current;
}

msp_rendor_mah_t construct_rendor_MAH(const battery_status_s &battery_status)
{
	msp_rendor_mah_t mah{};

	// Position will be set by caller
	mah.screenYPosition = 0;
	mah.screenXPosition = 0;

	if (battery_status.connected) {
		uint16_t mah_drawn = (uint16_t)battery_status.discharged_mah;
		char temp[12];
		int len = snprintf(temp, sizeof(temp), "%u", mah_drawn);
		if (len > 11) len = 11;  // Reserve space for SYM_MAH (buffer size 12)
		memcpy(mah.str, temp, len);
		mah.str[len] = '\x07';  // SYM_MAH
		mah.str_length = len + 1; // Dynamic: "5⚡"=2, "50⚡"=3, "1500⚡"=5
	} else {
		memcpy(mah.str, "----", 4);
		mah.str[4] = '\x07';  // SYM_MAH always displayed
		mah.str_length = 5; // "----⚡"=5
	}

	mah.iconAttrs = 0x00;

	return mah;
}

msp_rendor_stall_warning_t construct_rendor_STALL_WARNING(const airspeed_validated_s &airspeed_validated, float fw_airspd_min, bool test_mode)
{
	msp_rendor_stall_warning_t stall_warn{};

	// Position will be set by caller
	stall_warn.screenYPosition = 0;
	stall_warn.screenXPosition = 0;

	// Check if airspeed is below minimum (stall warning)
	float current_airspeed = airspeed_validated.true_airspeed_m_s;

	if (test_mode || (current_airspeed < fw_airspd_min && fw_airspd_min > 0.0f)) {
		// Test mode OR below stall speed - show red blinking alert (iconAttrs = 0x43 from struct default)
		memcpy(stall_warn.str, "STALL", 5);
		stall_warn.str_length = 5;
	} else {
		// Above stall speed - don't display
		stall_warn.iconAttrs = 0x00;
		memset(stall_warn.str, ' ', 5);
		stall_warn.str_length = 0; // Don't send if not displaying
	}

	return stall_warn;
}

msp_rendor_alt_warning_t construct_rendor_ALT_MAX_WARNING(float current_altitude, float max_altitude, bool force_test)
{
	msp_rendor_alt_warning_t alt_warn{};

	// Position will be set by caller
	alt_warn.screenYPosition = 0;
	alt_warn.screenXPosition = 0;

	if (force_test || (max_altitude > 0.0f && current_altitude > max_altitude)) {
		// Above max altitude - show orange blinking alert (iconAttrs = 0x42 from struct default)
		char temp[32];
		int len = snprintf(temp, sizeof(temp), "AB0VE %dM DESCEND N0W", (int)max_altitude);
		if (len > 26) len = 26;
		memcpy(alt_warn.str, temp, len);
		alt_warn.str_length = len; // Dynamic length
	} else {
		// Below max altitude - don't display
		alt_warn.iconAttrs = 0x00;
		alt_warn.str[0] = '\0';
		alt_warn.str_length = 0;
	}

	return alt_warn;
}

msp_rendor_g_meter_t construct_rendor_G_METER(const sensor_combined_s &sensor_combined, float max_g)
{
	msp_rendor_g_meter_t g_meter;

	// Calculate total G-force from accelerometer data
	// sensor_combined.accelerometer_m_s2[0] = X (forward)
	// sensor_combined.accelerometer_m_s2[1] = Y (right)
	// sensor_combined.accelerometer_m_s2[2] = Z (down)

	float accel_x = sensor_combined.accelerometer_m_s2[0];
	float accel_y = sensor_combined.accelerometer_m_s2[1];
	float accel_z = sensor_combined.accelerometer_m_s2[2];

	// Calculate total acceleration magnitude
	float total_accel = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);

	// Convert to G-force (divide by standard gravity 9.80665 m/s²)
	float g_force = total_accel / 9.80665f;

	// Clamp to reasonable range
	if (g_force < 0.0f) g_force = 0.0f;
	if (g_force > 20.0f) g_force = 20.0f;

	// Determine color based on max_g parameter
	if (max_g > 0.0f) {
		if (g_force >= max_g) {
			g_meter.iconAttrs = 0x03; // Red: over limit
		} else if (g_force >= max_g * 0.8f) {
			g_meter.iconAttrs = 0x02; // Orange: close to limit (>80%)
		} else {
			g_meter.iconAttrs = 0x00; // White: normal
		}
	} else {
		g_meter.iconAttrs = 0x00; // White if max_g is 0 (disabled)
	}

	// Format the string: "G X.X"
	char temp[8];
	snprintf(temp, sizeof(temp), "G %.1f", (double)g_force);

	// Copy to rendor structure
	size_t len = strlen(temp);
	if (len > sizeof(g_meter.str)) {
		len = sizeof(g_meter.str);
	}
	memcpy(g_meter.str, temp, len);
	g_meter.str_length = (uint8_t)len;

	return g_meter;
}

msp_rendor_pullpot_t construct_rendor_PULL_POT(const airspeed_validated_s &airspeed_validated,
		const sensor_combined_s &sensor_combined, int32_t source, float vs_m_s, float csv_gmax)
{
	msp_rendor_pullpot_t pull_pot{};

	// Position will be set by caller
	pull_pot.screenYPosition = 0;
	pull_pot.screenXPosition = 0;

	// Default safe visual state
	pull_pot.iconAttrs = 0x01; // Green
	memcpy(pull_pot.str, "PP: 0%", 6);
	pull_pot.str_length = 6;

	const float tas = airspeed_validated.true_airspeed_m_s;
	const float accel_x = sensor_combined.accelerometer_m_s2[0];
	const float accel_y = sensor_combined.accelerometer_m_s2[1];
	const float accel_z = sensor_combined.accelerometer_m_s2[2];

	if (!PX4_ISFINITE(tas) || tas < 0.0f
	    || !PX4_ISFINITE(accel_x) || !PX4_ISFINITE(accel_y) || !PX4_ISFINITE(accel_z)) {
		return pull_pot;
	}

	const float total_accel = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
	const float g_nrml = total_accel / CONSTANTS_ONE_G;
	float g_limit = NAN;

	if (source == 1 && PX4_ISFINITE(csv_gmax) && csv_gmax > 1.0f) {
		g_limit = csv_gmax;
	} else {
		if (PX4_ISFINITE(vs_m_s) && vs_m_s > 0.0f) {
			const float nnorm = 1.0f / (vs_m_s * vs_m_s);
			g_limit = nnorm * tas * tas;
		}
	}

	float pull_percent = 0.0f;
	const float denom = g_limit - 1.0f;

	if (PX4_ISFINITE(g_limit) && denom > 0.0f) {
		pull_percent = ((g_nrml - 1.0f) / denom) * 100.0f;
	}

	if (!PX4_ISFINITE(pull_percent) || pull_percent < 0.0f) {
		pull_percent = 0.0f;
	}

	if (pull_percent >= 90.0f) {
		pull_pot.iconAttrs = 0x03; // Red
	} else if (pull_percent >= 75.0f) {
		pull_pot.iconAttrs = 0x02; // Orange
	} else {
		pull_pot.iconAttrs = 0x01; // Green
	}

	if (pull_percent > 200.0f) {
		memcpy(pull_pot.str, "PP:>200%", 8);
		pull_pot.str_length = 8;
	} else {
		char temp[12];
		int len = snprintf(temp, sizeof(temp), "PP: %.0f%%", (double)pull_percent);

		if (len < 0) {
			memcpy(pull_pot.str, "PP: 0%", 6);
			pull_pot.str_length = 6;
		} else {
			if (len > static_cast<int>(sizeof(pull_pot.str))) {
				len = sizeof(pull_pot.str);
			}

			memcpy(pull_pot.str, temp, len);
			pull_pot.str_length = static_cast<uint8_t>(len);
		}
	}

	return pull_pot;
}

msp_rendor_throttle_t construct_rendor_THROTTLE(const input_rc_s &input_rc)
{
	msp_rendor_throttle_t throttle{};

	// Position will be set by caller
	throttle.screenYPosition = 0;
	throttle.screenXPosition = 0;

	int throttle_percent = 0;

	if (input_rc.timestamp > 0 && input_rc.channel_count > 2) {
		const float thr_raw = input_rc.values[2]; // RC throttle channel (1000..2000)
		const float thr_norm = (thr_raw - 1000.0f) / 10.0f;
		throttle_percent = math::constrain((int)thr_norm, 0, 100);
	}

	int len = snprintf(throttle.str, sizeof(throttle.str), "%d%%", throttle_percent);

	if (len < 0) {
		memcpy(throttle.str, "0%", 2);
		throttle.str_length = 2;
	} else {
		if (len > static_cast<int>(sizeof(throttle.str))) {
			len = sizeof(throttle.str);
		}

		throttle.str_length = static_cast<uint8_t>(len);
	}

	return throttle;
}


} // namespace msp_osd

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

/* Implementation of MessageDisplay class.
 */

#include <string.h>
#include "MessageDisplay.hpp"

namespace msp_osd
{

void MessageDisplay::set_message(const char *string)
{
	if (strcmp(message, string) != 0) {
		message[MSG_BUFFER_SIZE - 1] = '\0';
		strncpy(message, string, MSG_BUFFER_SIZE - 1);
		index = 0; // Reset scroll on new message
		last_update_ = 0;
	}
}

void MessageDisplay::get(char *output, int max_len, uint64_t current_time)
{
	int msg_len = strlen(message);

	if (msg_len == 0 || max_len <= 0) {
		output[0] = '\0';
		return;
	}

	// Message fits, no scrolling
	if (msg_len <= max_len) {
		strncpy(output, message, max_len);
		output[max_len] = '\0';
		return;
	}

	// Message too long, scroll it
	uint64_t dt = current_time - last_update_;

	if ((index == 0 && dt >= dwell_) || (index > 0 && dt >= period_)) {
		// Update scroll position
		index++;
		if (index > msg_len - max_len) {
			index = 0; // Loop back
		}
		last_update_ = current_time;
	}

	// Copy visible portion
	strncpy(output, message + index, max_len);
	output[max_len] = '\0';
}

} // namespace msp_osd

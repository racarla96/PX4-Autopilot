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
 * @file SafetyMonitor.cpp
 */

#include "SafetyMonitor.hpp"

#include <math.h>

namespace traj_control
{

const char *tripText(Trip trip)
{
	switch (trip) {
	case Trip::None: return "none";

	case Trip::InputsLost: return "state estimate or reference lost";

	case Trip::ErrorTooLarge: return "position error too large";

	case Trip::TooFar: return "too far from the entry point";

	case Trip::TooHigh: return "too high above the entry point";

	case Trip::TiltTooLarge: return "tilt too large";

	case Trip::ControllerFailure: return "controller failure";
	}

	return "unknown";
}

void SafetyMonitor::reset()
{
	*this = SafetyMonitor();
}

float SafetyMonitor::held(bool condition, float now_s, float &start)
{
	if (!condition) {
		start = -1.f;
		return 0.f;
	}

	if (start < 0.f) {
		start = now_s;
	}

	return now_s - start;
}

Trip SafetyMonitor::update(float now_s, const SafetyLimits &limits, const SafetySample &s)
{
	if (_trip != Trip::None) {
		return _trip;
	}

	if (held(!s.inputs_ok, now_s, _inputs_bad_since) >= limits.input_timeout && !s.inputs_ok) {
		_trip = Trip::InputsLost;
		return _trip;
	}

	if (!s.inputs_ok) {
		return Trip::None; // nothing else can be evaluated without a valid state
	}

	if (!_have_entry) {
		for (int i = 0; i < 3; i++) {
			_entry[i] = s.position[i];
		}

		_have_entry = true;
	}

	if (limits.tilt_abort > 0.f && s.tilt > limits.tilt_abort) {
		_trip = Trip::TiltTooLarge;

	} else if (limits.distance_max > 0.f
		   && hypotf(s.position[0] - _entry[0], s.position[1] - _entry[1]) > limits.distance_max) {
		_trip = Trip::TooFar;

	} else if (limits.altitude_max > 0.f && s.position[2] - _entry[2] > limits.altitude_max) {
		_trip = Trip::TooHigh;

	} else if (limits.error_max > 0.f
		   && held(s.error_norm > limits.error_max, now_s, _error_since) >= limits.error_time && s.error_norm > limits.error_max) {
		_trip = Trip::ErrorTooLarge;

	} else if (held(!s.controller_ok, now_s, _failure_since) >= limits.failure_time && !s.controller_ok) {
		_trip = Trip::ControllerFailure;
	}

	return _trip;
}

} // namespace traj_control

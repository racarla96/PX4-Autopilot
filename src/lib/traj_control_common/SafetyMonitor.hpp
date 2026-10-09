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
 * @file SafetyMonitor.hpp
 *
 * Safety checks shared by the LAB and FL flight modes. Pure logic (no PX4 dependencies). The modes bypass PX4's attitude
 * and rate loops, so the monitor looks at the vehicle and the controller from the outside and asks to leave the mode
 * (to Hold) when something is wrong:
 *  - InputsLost: state estimate or reference missing, stale or invalid for longer than input_timeout;
 *  - ErrorTooLarge: position error above error_max for longer than error_time;
 *  - TooFar / TooHigh: horizontal distance or height above the point where the mode was entered beyond the limits;
 *  - TiltTooLarge: attitude tilt above tilt_abort (the vehicle is no longer flying within the model's validity);
 *  - ControllerFailure: the law could not produce an output (singular decoupling matrix, non finite values) for longer
 *    than failure_time.
 * A limit of 0 disables the corresponding check. Once tripped the monitor stays tripped until reset().
 */

#pragma once

#include <stdint.h>

namespace traj_control
{

enum class Trip : uint8_t {
	None = 0,
	InputsLost,
	ErrorTooLarge,
	TooFar,
	TooHigh,
	TiltTooLarge,
	ControllerFailure,
};

const char *tripText(Trip trip);

struct SafetyLimits {
	float error_max{3.f};        // position error [m]
	float error_time{0.3f};      // [s]
	float distance_max{20.f};    // horizontal distance from the entry point [m]
	float altitude_max{12.f};    // height above the entry point [m]
	float tilt_abort{1.2f};      // [rad] (about 69 deg)
	float input_timeout{0.5f};   // [s]
	float failure_time{0.2f};    // [s]
};

struct SafetySample {
	bool inputs_ok{true};
	bool controller_ok{true};
	float position[3] {};  // x, y, z (z up) [m]
	float error_norm{0.f}; // |reference - position| [m]
	float tilt{0.f};       // [rad]
};

class SafetyMonitor
{
public:
	/** Call on every entry into the mode. */
	void reset();

	/** Evaluate one sample taken at time now_s [s]. Returns the first reason found (also on later calls once tripped). */
	Trip update(float now_s, const SafetyLimits &limits, const SafetySample &sample);

	Trip trip() const { return _trip; }

private:
	/** Time [s] since the condition started, 0 if it just started, and the condition's start is cleared when it ends. */
	float held(bool condition, float now_s, float &start);

	Trip _trip{Trip::None};
	bool _have_entry{false};
	float _entry[3] {};
	float _inputs_bad_since{-1.f};
	float _error_since{-1.f};
	float _failure_since{-1.f};
};

} // namespace traj_control

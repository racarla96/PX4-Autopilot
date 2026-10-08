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
 * @file TrajectoryProfile.hpp
 *
 * Deterministic reference trajectories (hover / circle / figure-8) with analytic
 * position, velocity, acceleration and jerk. Pure math: no uORB, no PX4 dependencies,
 * so it can be unit tested on the host.
 *
 * All quantities are relative to the vehicle state captured at the start of the
 * trajectory (local NED frame). The path always starts at the start position with
 * zero velocity (smooth speed ramp-up) and, if a duration is set, ends at rest
 * (smooth speed ramp-down).
 */

#pragma once

namespace traj_generator
{

enum class TrajectoryType : int {
	Hover = 0,
	Circle = 1,
	Figure8 = 2,
};

struct ProfileParams {
	TrajectoryType type{TrajectoryType::Circle};
	float amplitude{1.f};      // circle radius / figure-8 half-width [m]
	float frequency{0.1f};     // cruise angular rate of the path parameter is 2*pi*frequency [Hz]
	float altitude{0.f};       // target altitude above local origin [m], <= 0: keep the altitude at start
	float altitude_time{5.f};  // duration of the altitude change [s]
	float duration{0.f};       // total duration [s], <= 0: run forever
	float ramp{5.f};           // speed ramp-up / ramp-down time [s], <= 0: no ramp
};

struct ProfileStart {
	float x{0.f}; // [m] NED
	float y{0.f}; // [m] NED
	float z{0.f}; // [m] NED (positive down)
	float yaw{0.f}; // [rad]
};

struct ProfileReference {
	float position[3] {};
	float velocity[3] {};
	float acceleration[3] {};
	float jerk[3] {};
	float yaw{0.f};
	float yaw_rate{0.f};
	bool finished{false};
};

/**
 * Evaluate the reference at time t (seconds since the start of the trajectory).
 * For a finite duration, t >= duration returns the final point at rest with finished = true.
 */
ProfileReference evaluate(const ProfileParams &params, const ProfileStart &start, float t);

} // namespace traj_generator

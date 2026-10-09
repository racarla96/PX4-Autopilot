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
 * trajectory (local NED frame). TrajectoryType::Paper is the reference of the LAB vs FL paper
 * (see evaluatePaper) and ignores the other fields of ProfileParams. The path always starts at the start position with
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
	Paper = 3,
	Paper2 = 4,
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

/** Constants of the paper's reference trajectory (first trajectory of the LAB vs FL study, Ts = 0.02 s). */
constexpr float kPaperRadius = 2.f;       // circle radius [m]
constexpr float kPaperOmega = 0.3f;       // angular rate of the circle and of the yaw reference [rad/s]
constexpr float kPaperClimbRate = 0.05f;  // climb rate during the circle [m/s]
constexpr float kPaperLegTime = 40.f;     // duration of each of the two legs [s]

/**
 * The paper's reference, relative to the start state. Leg 1 (40 s): circle of radius 2 m at 0.3 rad/s, climbing
 * 0.05 m/s, with the yaw reference turning at 0.3 rad/s. Leg 2 (40 s): hold x, y and yaw while descending back to
 * the start altitude. It has no ramp: the velocity steps at t = 0 and at t = 40 s, exactly as in the paper.
 */
ProfileReference evaluatePaper(const ProfileStart &start, float t);

/** Duration of the paper's second trajectory [s]. */
constexpr float kPaper2Duration = 35.f;

/**
 * The paper's second reference (waypoints of data/referencia_externa.mat), relative to the start state: seven 5 s
 * point-to-point moves, each a minimum-snap (7th order) polynomial that starts and ends at rest with zero acceleration
 * and jerk, one axis at a time, while the yaw alternates between 0 and pi:
 *   0-5 s climb 5 m, 5-10 s x +5 m, 10-15 s y +10 m, 15-20 s x -5 m, 20-25 s descend 2 m, 25-30 s x +5 m, 30-35 s descend
 *   3 m (back to the start altitude).
 * Velocities up to 4.4 m/s, yaw rate up to 1.4 rad/s. Matches the file to 1e-13 (the last move to 8 mm).
 */
ProfileReference evaluatePaper2(const ProfileStart &start, float t);

/**
 * Evaluate the reference at time t (seconds since the start of the trajectory).
 * For a finite duration, t >= duration returns the final point at rest with finished = true.
 */
ProfileReference evaluate(const ProfileParams &params, const ProfileStart &start, float t);

} // namespace traj_generator

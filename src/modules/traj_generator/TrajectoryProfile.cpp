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
 * @file TrajectoryProfile.cpp
 */

#include "TrajectoryProfile.hpp"

#include <math.h>

namespace traj_generator
{

namespace
{

constexpr float kTwoPi = 6.28318530717958647692f;

// Smooth speed factor r(t) in [0,1] and its first/second derivative, plus I(t) = integral of r.
struct SpeedProfile {
	float r{1.f};
	float rd{0.f};
	float rdd{0.f};
	float integral{0.f};
};

// r = 3u^2 - 2u^3 (u in [0,1]) as a function of normalized time u, T = ramp duration
// Returns ramp value and derivatives with respect to time, for increasing u (ramp-up).
SpeedProfile rampUp(float t, float T)
{
	const float u = t / T;
	SpeedProfile s;
	s.r = 3.f * u * u - 2.f * u * u * u;
	s.rd = (6.f * u - 6.f * u * u) / T;
	s.rdd = (6.f - 12.f * u) / (T * T);
	s.integral = T * (u * u * u - 0.5f * u * u * u * u);
	return s;
}

// Same shape mirrored: u = (D - t) / T decreases from 1 to 0 over the last T seconds.
SpeedProfile rampDown(float t, float D, float T)
{
	const float u = (D - t) / T;
	SpeedProfile s;
	s.r = 3.f * u * u - 2.f * u * u * u;
	s.rd = -(6.f * u - 6.f * u * u) / T;
	s.rdd = (6.f - 12.f * u) / (T * T);
	// 0.5T (up) + (D - 2T) (cruise) + 0.5T - T(u^3 - u^4/2) (down)
	s.integral = D - T - T * (u * u * u - 0.5f * u * u * u * u);
	return s;
}

SpeedProfile speedProfile(float t, float duration, float ramp)
{
	SpeedProfile s;

	if (ramp <= 0.f) {
		s.integral = t;
		return s;
	}

	if (t < ramp) {
		return rampUp(t, ramp);
	}

	if (duration > 0.f && t > duration - ramp) {
		return rampDown(t, duration, ramp);
	}

	s.integral = t - 0.5f * ramp;
	return s;
}

// Quintic smoothstep 10u^3 - 15u^4 + 6u^5 and its time derivatives, over duration T.
struct Smoothstep {
	float s{0.f};
	float sd{0.f};
	float sdd{0.f};
	float sddd{0.f};
};

Smoothstep smoothstep(float t, float T)
{
	Smoothstep out;

	if (t <= 0.f) {
		return out;
	}

	if (t >= T) {
		out.s = 1.f;
		return out;
	}

	const float u = t / T;
	out.s = u * u * u * (10.f - 15.f * u + 6.f * u * u);
	out.sd = 30.f * u * u * (1.f - 2.f * u + u * u) / T;
	out.sdd = (60.f * u - 180.f * u * u + 120.f * u * u * u) / (T * T);
	out.sddd = (60.f - 360.f * u + 360.f * u * u) / (T * T * T);
	return out;
}

// Path in the horizontal plane as a function of the phase phi, with the first three derivatives
// with respect to phi. All paths pass through (0,0) at phi = 0.
void path(TrajectoryType type, float amplitude, float phi, float p[2], float p1[2], float p2[2], float p3[2])
{
	const float s1 = sinf(phi);
	const float c1 = cosf(phi);
	const float s2 = sinf(2.f * phi);
	const float c2 = cosf(2.f * phi);
	const float R = amplitude;

	switch (type) {
	case TrajectoryType::Circle:
		// Circle of radius R whose centre is R north of the start point.
		p[0] = R * (c1 - 1.f);
		p[1] = R * s1;
		p1[0] = -R * s1;
		p1[1] = R * c1;
		p2[0] = -R * c1;
		p2[1] = -R * s1;
		p3[0] = R * s1;
		p3[1] = -R * c1;
		break;

	case TrajectoryType::Figure8:
		// Lemniscate of Gerono: x = R sin(phi), y = R sin(phi) cos(phi), centred on the start point.
		p[0] = R * s1;
		p[1] = 0.5f * R * s2;
		p1[0] = R * c1;
		p1[1] = R * c2;
		p2[0] = -R * s1;
		p2[1] = -2.f * R * s2;
		p3[0] = -R * c1;
		p3[1] = -4.f * R * c2;
		break;

	case TrajectoryType::Hover:
	default:
		for (int i = 0; i < 2; i++) {
			p[i] = p1[i] = p2[i] = p3[i] = 0.f;
		}

		break;
	}
}

} // namespace

ProfileReference evaluate(const ProfileParams &params, const ProfileStart &start, float t)
{
	ProfileReference ref;

	if (t < 0.f) {
		t = 0.f;
	}

	const bool finite = params.duration > 0.f;
	float ramp = params.ramp > 0.f ? params.ramp : 0.f;

	if (finite && ramp > 0.5f * params.duration) {
		ramp = 0.5f * params.duration;
	}

	if (finite && t >= params.duration) {
		ref.finished = true;
		t = params.duration;
	}

	// Phase of the path and its time derivatives
	SpeedProfile speed = speedProfile(t, params.duration, ramp);
	const float omega = kTwoPi * params.frequency;

	if (ref.finished) {
		// Hold the final point at rest
		speed.r = speed.rd = speed.rdd = 0.f;
	}

	const float phi = omega * speed.integral;
	const float phi1 = omega * speed.r;
	const float phi2 = omega * speed.rd;
	const float phi3 = omega * speed.rdd;

	float p[2], p1[2], p2[2], p3[2];
	path(params.type, params.amplitude, phi, p, p1, p2, p3);

	for (int i = 0; i < 2; i++) {
		ref.position[i] = p[i];
		ref.velocity[i] = p1[i] * phi1;
		ref.acceleration[i] = p2[i] * phi1 * phi1 + p1[i] * phi2;
		ref.jerk[i] = p3[i] * phi1 * phi1 * phi1 + 3.f * p2[i] * phi1 * phi2 + p1[i] * phi3;
	}

	ref.position[0] += start.x;
	ref.position[1] += start.y;

	// Vertical channel: quintic transition from the start altitude to the target altitude
	if (params.altitude > 0.f) {
		const float altitude_time = params.altitude_time > 0.1f ? params.altitude_time : 0.1f;
		const Smoothstep ss = smoothstep(t, altitude_time);
		const float dz = -params.altitude - start.z;
		ref.position[2] = start.z + dz * ss.s;
		ref.velocity[2] = dz * ss.sd;
		ref.acceleration[2] = dz * ss.sdd;
		ref.jerk[2] = dz * ss.sddd;

		if (ref.finished) {
			ref.velocity[2] = ref.acceleration[2] = ref.jerk[2] = 0.f;
		}

	} else {
		ref.position[2] = start.z;
	}

	ref.yaw = start.yaw;
	ref.yaw_rate = 0.f;

	return ref;
}

} // namespace traj_generator

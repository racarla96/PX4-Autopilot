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
 * @file LabControlLaw.cpp
 */

#include "LabControlLaw.hpp"

#include <math.h>

namespace lab_control
{

namespace
{

float wrapPi(float angle)
{
	return angle - 2.f * static_cast<float>(M_PI) * floorf((angle + static_cast<float>(M_PI)) / (2.f * static_cast<float>(M_PI)));
}

float limit(float &value, float limit_abs, bool &saturated)
{
	if (value > limit_abs) {
		value = limit_abs;
		saturated = true;

	} else if (value < -limit_abs) {
		value = -limit_abs;
		saturated = true;
	}

	return value;
}

} // namespace

Output compute(const Gains &gains, float mass, float Ts, float gravity, const State &state, const Reference &now,
	       const Reference &next, float tilt_max)
{
	const float *k = gains.k;
	Output out;

	// Intermediate references of the sacrificed variables (vx, vy, vz, psi_dot) and their errors:
	//   x2_ref_n = (x1_ref(i+1) - k_1 e1 - x1) / Ts,   delta_x2 = (1 - k_2) (x2_ref_n - x2)   [aprox = 1]
	float delta_v[3];

	for (int i = 0; i < 3; i++) {
		const float e = now.position[i] - state.position[i];
		const float v_ref_n = (next.position[i] - k[2 * i] * e - state.position[i]) / Ts;
		delta_v[i] = (1.f - k[2 * i + 1]) * (v_ref_n - state.velocity[i]);
	}

	const float e_psi = wrapPi(now.yaw - state.euler[2]);
	const float w_ref_n = (wrapPi(next.yaw - state.euler[2]) - k[10] * e_psi) / Ts;
	const float delta_w = (1.f - k[11]) * (w_ref_n - state.euler_rate[2]);

	// Roll and pitch references: phi_ref = atan(d4 / (d6 + g Ts)), theta_ref = atan(-d2 sin(phi_ref) / d4).
	// Since sin(phi_ref) / d4 = 1 / hypot(d4, d6 + g Ts), theta_ref has no singularity at d4 = 0.
	const float a = delta_v[2] + gravity * Ts;
	const float hyp = hypotf(delta_v[1], a);
	float phi_ref = atan2f(delta_v[1], a);
	float theta_ref = (hyp > 1e-6f) ? atanf(-delta_v[0] / hyp) : 0.f;

	limit(phi_ref, tilt_max, out.tilt_saturated);
	limit(theta_ref, tilt_max, out.tilt_saturated);
	out.phi_ref = phi_ref;
	out.theta_ref = theta_ref;

	const float x8_ref_n = (1.f - k[6]) * (phi_ref - state.euler[0]) / Ts;
	const float x10_ref_n = (1.f - k[8]) * (theta_ref - state.euler[1]) / Ts;
	const float delta_x8 = (1.f - k[7]) * (x8_ref_n - state.euler_rate[0]);
	const float delta_x10 = (1.f - k[9]) * (x10_ref_n - state.euler_rate[1]);

	const float u1 = -delta_v[0] * sinf(theta_ref);
	const float u2 = delta_v[1] * cosf(theta_ref) * sinf(phi_ref);
	const float u3 = a * cosf(theta_ref) * cosf(phi_ref);
	out.thrust = mass / Ts * (u1 + u2 + u3);

	out.euler_accel[0] = delta_x8 / Ts;
	out.euler_accel[1] = delta_x10 / Ts;
	out.euler_accel[2] = delta_w / Ts;

	return out;
}

} // namespace lab_control

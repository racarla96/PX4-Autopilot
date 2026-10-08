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
 * @file TrajControlMath.cpp
 */

#include "TrajControlMath.hpp"

#include <math.h>

#include <px4_platform_common/defines.h>

using matrix::Vector3f;

namespace traj_control
{

namespace
{
constexpr float kMinCosTheta = 1e-3f;
}

Vector3f nedToPaper(const Vector3f &v)
{
	return Vector3f(v(0), v(1), -v(2));
}

Vector3f eulerRatesToBodyRates(const Vector3f &euler, const Vector3f &euler_rate)
{
	const float sphi = sinf(euler(0));
	const float cphi = cosf(euler(0));
	const float stheta = sinf(euler(1));
	const float ctheta = cosf(euler(1));

	return Vector3f(euler_rate(0) - stheta * euler_rate(2),
			cphi * euler_rate(1) + sphi * ctheta * euler_rate(2),
			-sphi * euler_rate(1) + cphi * ctheta * euler_rate(2));
}

Vector3f bodyRatesToEulerRates(const Vector3f &euler, const Vector3f &body_rate)
{
	const float sphi = sinf(euler(0));
	const float cphi = cosf(euler(0));
	const float ctheta_raw = cosf(euler(1));
	const float ctheta = (fabsf(ctheta_raw) < kMinCosTheta) ? copysignf(kMinCosTheta, ctheta_raw) : ctheta_raw;
	const float ttheta = sinf(euler(1)) / ctheta;
	const float qr = sphi * body_rate(1) + cphi * body_rate(2);

	return Vector3f(body_rate(0) + ttheta * qr,
			cphi * body_rate(1) - sphi * body_rate(2),
			qr / ctheta);
}

Vector3f eulerAccelToBodyAccel(const Vector3f &euler, const Vector3f &euler_rate, const Vector3f &euler_accel)
{
	const float sphi = sinf(euler(0));
	const float cphi = cosf(euler(0));
	const float stheta = sinf(euler(1));
	const float ctheta = cosf(euler(1));
	const float phid = euler_rate(0);
	const float thetad = euler_rate(1);

	// omega = W(euler) * euler_rate  ->  alpha = W * euler_accel + dW/dt * euler_rate
	const Vector3f W_acc = eulerRatesToBodyRates(euler, euler_accel);

	const Vector3f Wdot_rate(-ctheta * thetad * euler_rate(2),
				 -sphi * phid * euler_rate(1) + (cphi * ctheta * phid - sphi * stheta * thetad) * euler_rate(2),
				 -cphi * phid * euler_rate(1) + (-sphi * ctheta * phid - cphi * stheta * thetad) * euler_rate(2));

	return W_acc + Wdot_rate;
}

float tiltAngle(const matrix::Quatf &q)
{
	// z component of the body z axis expressed in the world frame: R(2,2)
	const matrix::Dcmf R(q);
	const float c = fminf(fmaxf(R(2, 2), -1.f), 1.f);
	return acosf(c);
}

bool limitAbs(float &value, float limit)
{
	if (!PX4_ISFINITE(value)) {
		value = 0.f;
		return true;
	}

	if (value > limit) {
		value = limit;
		return true;
	}

	if (value < -limit) {
		value = -limit;
		return true;
	}

	return false;
}

Setpoints toSetpoints(const VehicleModel &model, const Limits &limits, const Vector3f &euler,
		      const Vector3f &euler_rate, const Command &command)
{
	Setpoints out;

	// Collective thrust: u = m*g corresponds to the hover thrust
	float thrust = command.thrust / (model.mass * kGravity) * model.hover_thrust;

	if (!PX4_ISFINITE(thrust)) {
		thrust = limits.thrust_min;
		out.thrust_saturated = true;
		out.torque_saturated = true;
		out.thrust_norm = thrust;
		out.thrust = Vector3f(0.f, 0.f, -thrust);
		return out;
	}

	if (thrust > limits.thrust_max) {
		thrust = limits.thrust_max;
		out.thrust_saturated = true;

	} else if (thrust < limits.thrust_min) {
		thrust = limits.thrust_min;
		out.thrust_saturated = true;
	}

	out.thrust_norm = thrust;
	out.thrust = Vector3f(0.f, 0.f, -thrust);

	// Angular acceleration limits (Euler angle accelerations, as in the paper)
	Vector3f euler_accel = command.euler_accel;

	for (int i = 0; i < 3; i++) {
		out.torque_saturated |= limitAbs(euler_accel(i), limits.alpha_max(i));
	}

	// Euler kinematics -> body angular rate and acceleration
	const Vector3f omega = eulerRatesToBodyRates(euler, euler_rate);
	const Vector3f alpha = eulerAccelToBodyAccel(euler, euler_rate, euler_accel);

	// J*alpha + omega x (J*omega), with J diagonal
	const Vector3f J_omega(model.inertia(0) * omega(0), model.inertia(1) * omega(1), model.inertia(2) * omega(2));
	const Vector3f torque = Vector3f(model.inertia(0) * alpha(0), model.inertia(1) * alpha(1), model.inertia(2) * alpha(2))
				+ omega.cross(J_omega);

	for (int i = 0; i < 3; i++) {
		float norm = torque(i) / model.torque_max(i);
		out.torque_saturated |= limitAbs(norm, 1.f);
		out.torque(i) = norm;
	}

	return out;
}

Vector3f extrapolate(const Vector3f &position, const Vector3f &velocity, const Vector3f &acceleration,
		     const Vector3f &jerk, float dt)
{
	return position + velocity * dt + acceleration * (0.5f * dt * dt) + jerk * (dt * dt * dt / 6.f);
}

bool isFresh(uint64_t now_us, uint64_t timestamp_us, uint64_t timeout_us)
{
	return timestamp_us != 0 && now_us >= timestamp_us && (now_us - timestamp_us) <= timeout_us;
}

} // namespace traj_control

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
 * @file FlControlLaw.cpp
 */

#include "FlControlLaw.hpp"

#include <math.h>

#include <cmath>

#include <matrix/math.hpp>

namespace fl_control
{

namespace
{

float wrapPi(float angle)
{
	return angle - 2.f * static_cast<float>(M_PI) * floorf((angle + static_cast<float>(M_PI)) / (2.f * static_cast<float>(M_PI)));
}

} // namespace

float FilteredDerivative::step(float u, float Ts, float T)
{
	if (!_initialized) {
		// Initial state inherited from the first input (no derivative spike), as the Simulink block does
		_initialized = true;
		_xf = u;
	}

	const float Te = fmaxf(T, kMinRatio * Ts);
	const float y = (u - _xf) / Te;
	_xf += Ts * (u - _xf) / Te;
	return y;
}

void decouplingMatrix(float mass, float u, const float euler[3], float G[4][4])
{
	const float sphi = sinf(euler[0]);
	const float cphi = cosf(euler[0]);
	const float stheta = sinf(euler[1]);
	const float ctheta = cosf(euler[1]);
	const float im = 1.f / mass;

	const float rows[4][4] = {
		{-stheta * im, 0.f, -ctheta * u * im, 0.f},
		{ctheta * sphi * im, ctheta * cphi * u * im, -stheta * sphi * u * im, 0.f},
		{ctheta * cphi * im, -sphi * ctheta * u * im, -stheta * cphi * u * im, 0.f},
		{0.f, 0.f, 0.f, 1.f},
	};

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			G[i][j] = rows[i][j];
		}
	}
}

void modelAccelerationAndJerk(float mass, float gravity, float u, float u_dot, const float euler[3], const float euler_rate[3],
			      float accel[3], float jerk[3])
{
	const float sphi = sinf(euler[0]);
	const float cphi = cosf(euler[0]);
	const float stheta = sinf(euler[1]);
	const float ctheta = cosf(euler[1]);
	const float phid = euler_rate[0];
	const float thetad = euler_rate[1];
	const float im = 1.f / mass;

	accel[0] = -u * stheta * im;
	accel[1] = u * ctheta * sphi * im;
	accel[2] = u * ctheta * cphi * im - gravity;

	jerk[0] = -(u_dot * stheta + u * ctheta * thetad) * im;
	jerk[1] = (u_dot * ctheta * sphi - u * stheta * thetad * sphi + u * ctheta * cphi * phid) * im;
	jerk[2] = (u_dot * ctheta * cphi - u * stheta * thetad * cphi - u * ctheta * sphi * phid) * im;
}

void driftTerms(float mass, float u, float u_dot, const float euler[3], const float euler_rate[3], bool exact_fz,
		float f[3])
{
	const float sphi = sinf(euler[0]);
	const float cphi = cosf(euler[0]);
	const float stheta = sinf(euler[1]);
	const float ctheta = cosf(euler[1]);
	const float phid = euler_rate[0];
	const float thetad = euler_rate[1];
	const float im = 1.f / mass;

	f[0] = -(2.f * u_dot * ctheta * thetad - u * stheta * thetad * thetad) * im;

	f[1] = (2.f * u_dot * (-stheta * thetad * sphi + ctheta * cphi * phid)
		+ u * (-ctheta * sphi * thetad * thetad - 2.f * stheta * cphi * thetad * phid - ctheta * sphi * phid * phid)) * im;

	// u_dot term of fz: exact derivative of u cos(theta) cos(phi), or as wired in the paper's Simulink model
	const float fz_udot = exact_fz ? 2.f * u_dot * (-stheta * thetad * cphi - ctheta * sphi * phid)
			      : -2.f * u_dot * (cphi * ctheta * thetad - sphi * ctheta * phid);

	f[2] = (fz_udot
		+ u * (-ctheta * cphi * thetad * thetad + 2.f * stheta * sphi * thetad * phid - ctheta * cphi * phid * phid)) * im;
}

void FlControlLaw::reset(float u0)
{
	_u = u0;
	_u_dot = 0.f;

	for (int i = 0; i < 3; i++) {
		_accel[i].reset();
		_jerk[i].reset();
	}
}

Output FlControlLaw::step(const Gains &gains, float mass, float Ts, float tau_filter, const State &state,
			  const Reference &ref)
{
	Output out;
	out.thrust = _u;

	// Linear controllers: error + velocity error - estimated acceleration - estimated jerk
	float v[4];
	float model_accel[3];
	float model_jerk[3];

	if (gains.model_derivatives) {
		modelAccelerationAndJerk(mass, kGravity, _u, _u_dot, state.euler, state.euler_rate, model_accel, model_jerk);
	}

	for (int i = 0; i < 3; i++) {
		float accel;
		float jerk;

		if (gains.model_derivatives) {
			accel = model_accel[i];
			jerk = model_jerk[i];

		} else {
			accel = _accel[i].step(state.velocity[i], Ts, tau_filter);
			jerk = _jerk[i].step(accel, Ts, tau_filter);
		}

		v[i] = gains.k[i][0] * (ref.position[i] - state.position[i]) + gains.k[i][1] * (ref.velocity[i] - state.velocity[i])
		       - gains.k[i][2] * accel - gains.k[i][3] * jerk;
	}

	v[3] = gains.k_yaw[0] * wrapPi(ref.yaw - state.euler[2]) + gains.k_yaw[1] * (ref.yaw_rate - state.euler_rate[2]);

	float G[4][4];
	float f[3];
	decouplingMatrix(mass, _u, state.euler, G);
	driftTerms(mass, _u, _u_dot, state.euler, state.euler_rate, gains.exact_fz, f);

	// [u_ddot, tau_phi, tau_theta, tau_psi]' = G^-1 (v - f)
	matrix::SquareMatrix<float, 4> Gm;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			Gm(i, j) = G[i][j];
		}
	}

	matrix::SquareMatrix<float, 4> Gi;
	matrix::Vector<float, 4> rhs;
	rhs(0) = v[0] - f[0];
	rhs(1) = v[1] - f[1];
	rhs(2) = v[2] - f[2];
	rhs(3) = v[3];

	bool ok = matrix::inv(Gm, Gi);
	matrix::Vector<float, 4> U = Gi * rhs;

	for (int i = 0; i < 4; i++) {
		ok = ok && std::isfinite(U(i));
	}

	if (!ok) {
		out.singular = true;
		return out; // inputs stay zero, the integrators hold u
	}

	out.thrust_ddot = U(0);
	out.euler_accel[0] = U(1);
	out.euler_accel[1] = U(2);
	out.euler_accel[2] = U(3);

	// Advance the thrust integrators (forward Euler, u_ddot held over the sample)
	_u += _u_dot * Ts;
	_u_dot += out.thrust_ddot * Ts;

	return out;
}

void FlControlLaw::limitThrust(float u_min, float u_max)
{
	if (_u > u_max) {
		_u = u_max;
		_u_dot = fminf(_u_dot, 0.f);

	} else if (_u < u_min) {
		_u = u_min;
		_u_dot = fmaxf(_u_dot, 0.f);
	}
}

} // namespace fl_control

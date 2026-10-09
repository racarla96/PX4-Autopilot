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

	// Heading compensation: the plant is a_world = R(psi) a_h with a_h the paper's horizontal acceleration (a function of u,
	// phi, theta). Rotating the world-frame position loops into the heading frame needs the rotation terms:
	//   a_world'' = R (a_h'' + 2 psi_dot J a_h' + psi_ddot J a_h - psi_dot^2 a_h),  J = [[0, -1], [1, 0]]
	// psi_ddot is the yaw input, which does not depend on the position loops, so it is computed first.
	const bool rotate = gains.heading_compensation && gains.model_derivatives;
	const float psi = state.euler[2];
	const float psi_dot = state.euler_rate[2];
	const float cpsi = rotate ? cosf(psi) : 1.f;
	const float spsi = rotate ? sinf(psi) : 0.f;

	v[3] = gains.k_yaw[0] * wrapPi(ref.yaw - state.euler[2]) + gains.k_yaw[1] * (ref.yaw_rate - state.euler_rate[2]);

	float accel_w[3] {};
	float jerk_w[3] {};

	for (int i = 0; i < 3; i++) {
		if (gains.model_derivatives) {
			accel_w[i] = model_accel[i];
			jerk_w[i] = model_jerk[i];

		} else {
			accel_w[i] = _accel[i].step(state.velocity[i], Ts, tau_filter);
			jerk_w[i] = _jerk[i].step(accel_w[i], Ts, tau_filter);
		}
	}

	if (rotate) {
		const float ah[2] = {model_accel[0], model_accel[1]};
		const float jh[2] = {model_jerk[0], model_jerk[1]};
		accel_w[0] = cpsi * ah[0] - spsi * ah[1];
		accel_w[1] = spsi * ah[0] + cpsi * ah[1];
		// a_w' = R (psi_dot J a_h + a_h')
		const float jx = -psi_dot * ah[1] + jh[0];
		const float jy = psi_dot * ah[0] + jh[1];
		jerk_w[0] = cpsi * jx - spsi * jy;
		jerk_w[1] = spsi * jx + cpsi * jy;
	}

	for (int i = 0; i < 3; i++) {
		v[i] = gains.k[i][0] * (ref.position[i] - state.position[i]) + gains.k[i][1] * (ref.velocity[i] - state.velocity[i])
		       - gains.k[i][2] * accel_w[i] - gains.k[i][3] * jerk_w[i];
	}

	if (rotate) {
		const float ah[2] = {model_accel[0], model_accel[1]};
		const float jh[2] = {model_jerk[0], model_jerk[1]};
		const float vh_x = cpsi * v[0] + spsi * v[1];
		const float vh_y = -spsi * v[0] + cpsi * v[1];
		// a_h'' = R^T v + psi_dot^2 a_h - 2 psi_dot J a_h' - psi_ddot J a_h
		v[0] = vh_x + psi_dot * psi_dot * ah[0] - 2.f * psi_dot * (-jh[1]) - v[3] * (-ah[1]);
		v[1] = vh_y + psi_dot * psi_dot * ah[1] - 2.f * psi_dot * jh[0] - v[3] * ah[0];
	}

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

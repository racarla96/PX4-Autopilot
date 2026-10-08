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
 * @file FlControlLaw.hpp
 *
 * FL (feedback linearization) trajectory controller of the LAB vs FL study (Simulink model
 * modelo_dinamico_FL.slx and controllers/FL_controller.m). Pure math, no PX4 dependencies.
 *
 * Plant (paper's frame: x North, y East, z Up):
 *   m x_ddot = -u sin(theta),  m y_ddot = u cos(theta) sin(phi),  m z_ddot = u cos(theta) cos(phi) - m g,
 *   phi_ddot = tau_phi, theta_ddot = tau_theta, psi_ddot = tau_psi.
 * x, y and z have relative degree 4 once the thrust u is made a state (dynamic extension, two integrators):
 * the inputs are u_ddot and the three angular accelerations, obtained from
 *   [u_ddot, tau_phi, tau_theta, tau_psi]' = G^-1 (v - f)
 * where v are the outputs of four linear controllers placed with pole placement (gains K1..K4) acting on the
 * position error, the velocity error and the estimated acceleration and jerk of the vehicle (derivatives
 * filtered as in the Simulink model, see FilteredDerivative).
 *
 * Validated against the Simulink model: with its plant (ode1, Ts = 0.02 s) this law reproduces the simulated
 * trajectory to machine precision. Two details of the study that a reader of the equations alone would miss:
 *  - the derivative filter has a time constant of max(T, 10 Ts), not T;
 *  - the paper's fz drift term does not match the exact derivative of m z_ddot = u cos(theta) cos(phi) - m g in its
 *    u_dot terms (Gains::exact_fz selects the exact one). The effect is small because u_dot is small.
 */

#pragma once

namespace fl_control
{

/**
 * Pole placement gains. Row i is [k_i1 k_i2 k_i3 k_i4] of channel x, y, z; k4 is [k_41, k_42] of the yaw channel.
 * Defaults: first trajectory of the study (build_controller_gains.m, tray = 1).
 */
struct Gains {
	float k[3][4] {{1.452f, 3.509f, 2.41f, 2.9f},
		{1.26f, 2.5f, 2.26f, 2.5f},
		{1.6698f, 3.509f, 2.59f, 2.9f}};
	float k_yaw[2] {0.5f, 1.5f};
	bool exact_fz{false};  // false: fz exactly as in the paper's Simulink model, true: exact derivative
};

struct State {
	float position[3] {};    // x, y, z (z up) [m]
	float velocity[3] {};    // [m/s]
	float euler[3] {};       // phi, theta, psi [rad]
	float euler_rate[3] {};  // [rad/s]
};

struct Reference {
	float position[3] {};
	float velocity[3] {};
	float yaw{0.f};
	float yaw_rate{0.f};
};

struct Output {
	float thrust{0.f};            // u [N] applied at this sample (state of the first integrator)
	float thrust_ddot{0.f};       // u_ddot [N/s^2]
	float euler_accel[3] {};      // tau_phi, tau_theta, tau_psi [rad/s^2]
	bool singular{false};         // G could not be inverted: all inputs are zero this sample
};

/**
 * Filtered derivative K s / (T s + 1) with K = 1, implemented as the Filtered Derivative block of the study:
 * y = (u - xf) / Te and xf_next = xf + Ts (u - xf) / Te (forward Euler), with Te = max(T, kMinRatio Ts) and
 * xf initialised to the first input.
 */
class FilteredDerivative
{
public:
	static constexpr float kMinRatio = 10.f;  // minimum ratio between the filter time constant and the sample time

	void reset() { _initialized = false; }
	float step(float u, float Ts, float T);

private:
	float _xf{0.f};
	bool _initialized{false};
};

/** Decoupling matrix G (4x4, row-major) for the inputs [u_ddot, tau_phi, tau_theta, tau_psi]. Exposed for testing. */
void decouplingMatrix(float mass, float u, const float euler[3], float G[4][4]);

/** Drift terms f of the fourth derivative of x, y, z (see Gains::exact_fz). Exposed for testing. */
void driftTerms(float mass, float u, float u_dot, const float euler[3], const float euler_rate[3], bool exact_fz,
		float f[3]);

class FlControlLaw
{
public:
	/** Restart the law at hover: u = u0 (m g), u_dot = 0 and clear the derivative filters. Call on every mode entry. */
	void reset(float u0);

	/**
	 * One step at sample time Ts. `tau_filter` is the derivative filter time constant (0.01 s in the study).
	 * Returns the inputs for this sample and then advances the thrust integrators with u_ddot held over Ts
	 * (forward Euler, as the Simulink integrators with ode1).
	 */
	Output step(const Gains &gains, float mass, float Ts, float tau_filter, const State &state, const Reference &ref);

	/** Keep the thrust state inside [u_min, u_max] [N] (anti-windup: u_dot is zeroed on contact). */
	void limitThrust(float u_min, float u_max);

	float thrust() const { return _u; }
	float thrustRate() const { return _u_dot; }

private:
	FilteredDerivative _accel[3];
	FilteredDerivative _jerk[3];
	float _u{0.f};
	float _u_dot{0.f};
};

} // namespace fl_control

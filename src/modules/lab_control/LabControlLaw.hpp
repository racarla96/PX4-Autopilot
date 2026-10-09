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
 * @file LabControlLaw.hpp
 *
 * LAB (Linear Algebra Based) trajectory controller of the LAB vs FL study, one step of the discrete law of
 * controllers/LAB_controller.m (one-step approximation, aprox = 1). Pure math, no PX4 dependencies.
 *
 * Everything is in the paper's frame: x North, y East, z Up, Euler angles phi / theta / psi, and the paper's
 * state ordering. The outputs are the paper's control inputs: collective thrust u [N] and the Euler angle
 * accelerations (tau_phi, tau_theta, tau_psi) [rad/s^2].
 */

#pragma once

namespace lab_control
{

/** k_1 .. k_12 of the paper: [x, vx, y, vy, z, vz, phi, phi_dot, theta, theta_dot, psi, psi_dot]. */
struct Gains {
	float k[12] {0.99f, 0.97f, 0.99f, 0.98f, 0.99f, 0.79f, 0.96f, 0.91f, 0.96f, 0.92f, 0.99f, 0.95f};
	bool heading_compensation{false};  // rotate the desired horizontal velocity change by the heading (see TCC_HDG_COMP)
};

struct State {
	float position[3] {};    // x, y, z (z up) [m]
	float velocity[3] {};    // [m/s]
	float euler[3] {};       // phi, theta, psi [rad]
	float euler_rate[3] {};  // [rad/s]
};

struct Reference {
	float position[3] {};    // x, y, z (z up) [m]
	float yaw{0.f};          // [rad]
};

struct Output {
	float thrust{0.f};            // u [N]
	float euler_accel[3] {};      // tau_phi, tau_theta, tau_psi [rad/s^2]
	float phi_ref{0.f};           // intermediate roll / pitch references [rad] (after the tilt limit)
	float theta_ref{0.f};
	bool tilt_saturated{false};   // phi_ref or theta_ref hit tilt_max
};

/**
 * One step. `now` is the reference at this sample (x_ref(i)) and `next` the reference one sample ahead
 * (x_ref(i+1)). Yaw differences are wrapped to +-pi (the paper's yaw grows without wrapping).
 * phi_ref and theta_ref are limited to +-tilt_max.
 */
Output compute(const Gains &gains, float mass, float Ts, float gravity, const State &state, const Reference &now,
	       const Reference &next, float tilt_max);

} // namespace lab_control

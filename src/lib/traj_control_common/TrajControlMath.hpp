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
 * @file TrajControlMath.hpp
 *
 * Pure math shared by the LAB and FL trajectory controllers: frame conversion, Euler kinematics,
 * conversion of the paper's control inputs into PX4 normalized thrust / torque setpoints,
 * saturations and timeouts. No uORB or parameter dependencies, so it is unit tested on the host.
 *
 * Conventions
 * -----------
 * The paper's model (x_ddot = -u sin(theta)/m, y_ddot = u cos(theta) sin(phi)/m,
 * z_ddot = u cos(theta) cos(phi)/m - g) matches PX4's aeronautical conventions with the z axis
 * flipped: x = North, y = East, z = Up (z_paper = -z_NED), and phi / theta / psi are PX4's roll / pitch /
 * yaw (ZYX Euler angles of vehicle_attitude). The paper's control inputs are the collective thrust u [N]
 * and the Euler angle accelerations (phi_ddot, theta_ddot, psi_ddot) [rad/s^2].
 */

#pragma once

#include <stdint.h>

#include <matrix/math.hpp>

namespace traj_control
{

/** Gravity used by the paper's model [m/s^2]. */
constexpr float kGravity = 9.81f;

/** Physical model used to scale the paper's control inputs into PX4's normalized setpoints. */
struct VehicleModel {
	float mass{1.5f};                                   // [kg]
	matrix::Vector3f inertia{0.02f, 0.02f, 0.04f};      // diagonal of the inertia matrix Ixx, Iyy, Izz [kg m^2]
	matrix::Vector3f torque_max{3.3f, 3.3f, 0.47f};        // physical torque that corresponds to a normalized torque of 1 [N m]
	float hover_thrust{0.5f};                           // normalized collective thrust that balances m*g
};

struct Limits {
	float thrust_min{0.1f};                             // normalized collective thrust [0,1]
	float thrust_max{0.9f};
	matrix::Vector3f alpha_max{10.f, 10.f, 1.f};        // Euler angle acceleration limits phi, theta, psi [rad/s^2]
	float tilt_max{0.61f};                              // maximum tilt angle [rad]
};

/** The paper's control inputs [u, tau_phi, tau_theta, tau_psi]. */
struct Command {
	float thrust{0.f};                                  // collective thrust u [N]
	matrix::Vector3f euler_accel{};                     // phi_ddot, theta_ddot, psi_ddot [rad/s^2]
};

/** Setpoints for vehicle_thrust_setpoint / vehicle_torque_setpoint, plus the saturation flags. */
struct Setpoints {
	matrix::Vector3f thrust{};                          // body FRD, normalized [-1,1]; z = -collective thrust
	matrix::Vector3f torque{};                          // body FRD, normalized [-1,1]
	float thrust_norm{0.f};                             // normalized collective thrust [0,1]
	bool thrust_saturated{false};
	bool torque_saturated{false};                       // angular acceleration limit or normalized torque limit hit
};

/** Local NED <-> paper frame (x North, y East, z Up). The conversion is its own inverse. */
matrix::Vector3f nedToPaper(const matrix::Vector3f &v);
inline matrix::Vector3f paperToNed(const matrix::Vector3f &v) { return nedToPaper(v); }

/** Body angular rate (p, q, r) from Euler angle rates (phi_dot, theta_dot, psi_dot). */
matrix::Vector3f eulerRatesToBodyRates(const matrix::Vector3f &euler, const matrix::Vector3f &euler_rate);

/** Euler angle rates from body angular rate. Near theta = +-90 deg cos(theta) is limited to keep the result finite. */
matrix::Vector3f bodyRatesToEulerRates(const matrix::Vector3f &euler, const matrix::Vector3f &body_rate);

/** Body angular acceleration from Euler angle acceleration, including the d/dt of the kinematic matrix. */
matrix::Vector3f eulerAccelToBodyAccel(const matrix::Vector3f &euler, const matrix::Vector3f &euler_rate,
				       const matrix::Vector3f &euler_accel);

/** Angle between the body z axis and the vertical [rad]. */
float tiltAngle(const matrix::Quatf &q);

/** Limit value to [-limit, limit]. Returns true if it was limited. A non finite value becomes 0 and counts as limited. */
bool limitAbs(float &value, float limit);

/**
 * Convert the paper's control inputs into normalized thrust and torque setpoints.
 * torque_body = J * alpha_body + omega x (J * omega), then divided by torque_max.
 * A non finite command yields minimum thrust and zero torque, flagged as saturated.
 */
Setpoints toSetpoints(const VehicleModel &model, const Limits &limits, const matrix::Vector3f &euler,
		      const matrix::Vector3f &euler_rate, const Command &command);

/** Position dt seconds ahead from the third order Taylor expansion with velocity, acceleration and jerk. */
matrix::Vector3f extrapolate(const matrix::Vector3f &position, const matrix::Vector3f &velocity,
			     const matrix::Vector3f &acceleration, const matrix::Vector3f &jerk, float dt);

/**
 * Hover thrust to use: the live estimate of mc_hover_thrust_estimator if it is valid and plausible, otherwise the
 * parameter. The real hover thrust depends on the vehicle and its thrust curve (the x500 of Gazebo hovers at 0.73
 * with a parameter of 0.60), and a wrong value makes the controllers hold altitude with a permanent offset.
 */
float selectHoverThrust(float estimate, bool estimate_valid, float param_value);

/** True if a message with this timestamp is not older than the timeout. A zero timestamp is never fresh. */
bool isFresh(uint64_t now_us, uint64_t timestamp_us, uint64_t timeout_us);

} // namespace traj_control

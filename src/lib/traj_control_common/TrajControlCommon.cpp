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
 * @file TrajControlCommon.cpp
 */

#include "TrajControlCommon.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

namespace traj_control
{

TrajControlCommon::TrajControlCommon(ModuleParams *parent) :
	ModuleParams(parent)
{
	updateParams();
}

void TrajControlCommon::updateParams()
{
	ModuleParams::updateParams();

	_model.mass = _param_tcc_mass.get();
	_model.inertia = matrix::Vector3f(_param_tcc_ixx.get(), _param_tcc_iyy.get(), _param_tcc_izz.get());
	_model.torque_max = matrix::Vector3f(_param_tcc_tq_max_xy.get(), _param_tcc_tq_max_xy.get(), _param_tcc_tq_max_z.get());
	_model.hover_thrust = _param_mpc_thr_hover.get();

	_limits.thrust_min = _param_tcc_thr_min.get();
	_limits.thrust_max = math::max(_param_tcc_thr_max.get(), _param_tcc_thr_min.get());
	_limits.alpha_max = matrix::Vector3f(_param_tcc_alpha_xy.get(), _param_tcc_alpha_xy.get(), _param_tcc_alpha_z.get());
	_limits.tilt_max = math::radians(_param_tcc_tilt_max.get());
}

void TrajControlCommon::publishSetpoints(const Setpoints &setpoints, hrt_abstime timestamp, hrt_abstime timestamp_sample)
{
	vehicle_thrust_setpoint_s thrust{};
	thrust.timestamp = timestamp;
	thrust.timestamp_sample = timestamp_sample;
	setpoints.thrust.copyTo(thrust.xyz);
	_thrust_pub.publish(thrust);

	vehicle_torque_setpoint_s torque{};
	torque.timestamp = timestamp;
	torque.timestamp_sample = timestamp_sample;
	setpoints.torque.copyTo(torque.xyz);
	_torque_pub.publish(torque);
}

void TrajControlCommon::publishIdle(hrt_abstime timestamp)
{
	Setpoints idle;
	idle.thrust_norm = _limits.thrust_min;
	idle.thrust = matrix::Vector3f(0.f, 0.f, -_limits.thrust_min);
	publishSetpoints(idle, timestamp, timestamp);
}

void TrajControlCommon::publishStatus(uint8_t controller, bool active, hrt_abstime timestamp,
				      const trajectory_reference_s &ref, const vehicle_local_position_s &lpos, float yaw,
				      const Setpoints &setpoints, bool tilt_saturated, const float *q_d)
{
	traj_control_status_s status{};
	status.timestamp = timestamp;
	status.controller = controller;
	status.active = active;

	const float pos[3] = {lpos.x, lpos.y, lpos.z};
	const float vel[3] = {lpos.vx, lpos.vy, lpos.vz};

	for (int i = 0; i < 3; i++) {
		status.position_ref[i] = ref.position[i];
		status.velocity_ref[i] = ref.velocity[i];
		status.position[i] = pos[i];
		status.velocity[i] = vel[i];
		status.position_error[i] = ref.position[i] - pos[i];
		status.velocity_error[i] = ref.velocity[i] - vel[i];
	}

	status.yaw_error = matrix::wrap_pi(ref.yaw - yaw);
	status.thrust = setpoints.thrust_norm;

	for (int i = 0; i < 4; i++) {
		status.q_d[i] = q_d ? q_d[i] : NAN;
	}

	status.thrust_saturated = setpoints.thrust_saturated;
	status.torque_saturated = setpoints.torque_saturated;
	status.tilt_saturated = tilt_saturated;

	if (setpoints.thrust_saturated || setpoints.torque_saturated || tilt_saturated) {
		_saturation_count++;
	}

	status.saturation_count = _saturation_count;
	_status_pub.publish(status);
}

} // namespace traj_control

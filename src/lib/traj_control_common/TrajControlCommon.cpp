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
#include <commander/px4_custom_mode.h>
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
	_model.hover_thrust = _hover_latched ? _hover_thrust_latched : _param_mpc_thr_hover.get();

	_limits.thrust_min = _param_tcc_thr_min.get();
	_limits.thrust_max = math::max(_param_tcc_thr_max.get(), _param_tcc_thr_min.get());
	_limits.alpha_max = matrix::Vector3f(_param_tcc_alpha_xy.get(), _param_tcc_alpha_xy.get(), _param_tcc_alpha_z.get());
	_limits.tilt_max = math::radians(_param_tcc_tilt_max.get());
}

void TrajControlCommon::latchHoverThrust()
{
	hover_thrust_estimate_s estimate{};
	const bool have = _hover_thrust_estimate_sub.copy(&estimate);

	_hover_thrust_latched = selectHoverThrust(have ? estimate.hover_thrust : NAN, have && estimate.valid,
				_param_mpc_thr_hover.get());
	_hover_latched = true;
	_model.hover_thrust = _hover_thrust_latched;
	_safety.reset();
	_last_hold_request = 0;
}

bool TrajControlCommon::readInputs(hrt_abstime now, Inputs &in)
{
	vehicle_attitude_s attitude{};
	vehicle_angular_velocity_s angular_velocity{};

	if (!_local_position_sub.copy(&in.local_position) || !_attitude_sub.copy(&attitude)
	    || !_angular_velocity_sub.copy(&angular_velocity) || !_reference_sub.copy(&in.reference)) {
		return false;
	}

	const hrt_abstime est_timeout = estimateTimeout();

	if (!isFresh(now, in.local_position.timestamp, est_timeout) || !isFresh(now, attitude.timestamp, est_timeout)
	    || !isFresh(now, angular_velocity.timestamp, est_timeout)
	    || !isFresh(now, in.reference.timestamp, referenceTimeout())) {
		return false;
	}

	if (!in.reference.valid || !in.local_position.xy_valid || !in.local_position.z_valid
	    || !in.local_position.v_xy_valid || !in.local_position.v_z_valid
	    || !in.local_position.heading_good_for_control) {
		return false;
	}

	const matrix::Eulerf euler(matrix::Quatf(attitude.q));
	in.euler = matrix::Vector3f(euler.phi(), euler.theta(), euler.psi());
	in.euler_rate = bodyRatesToEulerRates(in.euler, matrix::Vector3f(angular_velocity.xyz));

	in.position = nedToPaper(matrix::Vector3f(in.local_position.x, in.local_position.y, in.local_position.z));
	in.velocity = nedToPaper(matrix::Vector3f(in.local_position.vx, in.local_position.vy, in.local_position.vz));

	const matrix::Vector3f ref_p = nedToPaper(matrix::Vector3f(in.reference.position));
	const matrix::Vector3f ref_v = nedToPaper(matrix::Vector3f(in.reference.velocity));
	const matrix::Vector3f ref_a = nedToPaper(matrix::Vector3f(in.reference.acceleration));
	const matrix::Vector3f ref_j = nedToPaper(matrix::Vector3f(in.reference.jerk));
	const float Ts = samplePeriod();

	in.ref_position = ref_p;
	in.ref_velocity = ref_v;
	in.ref_position_next = extrapolate(ref_p, ref_v, ref_a, ref_j, Ts);
	in.ref_yaw = in.reference.yaw;
	in.ref_yaw_next = in.reference.yaw + in.reference.yaw_rate * Ts;
	in.ref_yaw_rate = in.reference.yaw_rate;
	in.timestamp_sample = angular_velocity.timestamp_sample;

	return PX4_ISFINITE(in.euler(0)) && PX4_ISFINITE(in.euler(1)) && PX4_ISFINITE(in.euler(2))
	       && PX4_ISFINITE(in.position(0)) && PX4_ISFINITE(in.velocity(0)) && PX4_ISFINITE(in.ref_position(0));
}

bool TrajControlCommon::checkSafety(hrt_abstime now, const Inputs *in, bool controller_ok)
{
	SafetyLimits limits;
	limits.error_max = _param_tcc_err_max.get();
	limits.distance_max = _param_tcc_dist_max.get();
	limits.altitude_max = _param_tcc_alt_max.get();
	limits.tilt_abort = math::radians(_param_tcc_tilt_abort.get());
	limits.input_timeout = _param_tcc_fail_tout.get();

	SafetySample sample;
	sample.inputs_ok = (in != nullptr);
	sample.controller_ok = controller_ok;

	if (in) {
		for (int i = 0; i < 3; i++) {
			sample.position[i] = in->position(i);
		}

		sample.error_norm = (in->ref_position - in->position).norm();
		sample.tilt = acosf(math::constrain(cosf(in->euler(0)) * cosf(in->euler(1)), -1.f, 1.f));
	}

	const Trip before = _safety.trip();
	const Trip trip = _safety.update(static_cast<float>(now) * 1e-6f, limits, sample);

	if (trip == Trip::None) {
		return false;
	}

	if (before == Trip::None) {
		mavlink_log_critical(&_mavlink_log_pub, "Trajectory mode: %s, switching to Hold", tripText(trip));
		PX4_WARN("safety trip: %s (tilt %.2f rad, error %.2f m, position %.1f %.1f %.1f)", tripText(trip), (double)sample.tilt,
			 (double)sample.error_norm, (double)sample.position[0], (double)sample.position[1], (double)sample.position[2]);
	}

	if (now - _last_hold_request > 500000) {
		_last_hold_request = now;
		vehicle_command_s command{};
		command.timestamp = now;
		command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		command.param1 = 1.f;
		command.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
		command.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		command.target_system = 1;
		command.target_component = 1;
		command.source_system = 1;
		command.source_component = 1;
		_command_pub.publish(command);
	}

	return true;
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

void TrajControlCommon::publishNeutral(hrt_abstime timestamp)
{
	Setpoints neutral;
	neutral.thrust_norm = math::constrain(_model.hover_thrust, _limits.thrust_min, _limits.thrust_max);
	neutral.thrust = matrix::Vector3f(0.f, 0.f, -neutral.thrust_norm);
	publishSetpoints(neutral, timestamp, timestamp);
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

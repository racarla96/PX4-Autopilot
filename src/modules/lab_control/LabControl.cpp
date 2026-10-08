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
 * @file LabControl.cpp
 */

#include "LabControl.hpp"

#include <lib/mathlib/mathlib.h>

using namespace time_literals;

LabControl::LabControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

bool LabControl::init()
{
	schedule();
	return true;
}

void LabControl::schedule()
{
	_interval_us = math::max(static_cast<uint32_t>(_common.samplePeriod() * 1e6f), static_cast<uint32_t>(1000));
	ScheduleOnInterval(_interval_us);
}

void LabControl::updateParams()
{
	ModuleParams::updateParams();
}

lab_control::Gains LabControl::gains() const
{
	lab_control::Gains g;
	const float values[12] = {_param_k1.get(), _param_k2.get(), _param_k3.get(), _param_k4.get(), _param_k5.get(), _param_k6.get(),
				  _param_k7.get(), _param_k8.get(), _param_k9.get(), _param_k10.get(), _param_k11.get(), _param_k12.get()
				 };

	for (int i = 0; i < 12; i++) {
		g.k[i] = values[i];
	}

	return g;
}

void LabControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();

		if (static_cast<uint32_t>(_common.samplePeriod() * 1e6f) != _interval_us) {
			schedule();
		}
	}

	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		_nav_state = status.nav_state;
	}

	if (_nav_state != vehicle_status_s::NAVIGATION_STATE_LAB) {
		_was_active = false;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!_was_active) {
		_common.latchHoverThrust();
		_common.resetSaturationCount();
		_was_active = true;
	}

	traj_control::Inputs in;

	if (!_common.readInputs(now, in)) {
		// Mode active but inputs missing or stale: keep publishing a neutral setpoint until the safety logic leaves the mode
		_common.publishNeutral(now);
		_last_input_failure = now;
		return;
	}

	const float Ts = _common.samplePeriod();

	lab_control::State state;
	lab_control::Reference ref_now;
	lab_control::Reference ref_next;

	for (int i = 0; i < 3; i++) {
		state.position[i] = in.position(i);
		state.velocity[i] = in.velocity(i);
		state.euler[i] = in.euler(i);
		state.euler_rate[i] = in.euler_rate(i);
		ref_now.position[i] = in.ref_position(i);
		ref_next.position[i] = in.ref_position_next(i);
	}

	ref_now.yaw = in.ref_yaw;
	ref_next.yaw = in.ref_yaw_next;

	const lab_control::Output out = lab_control::compute(gains(), _common.model().mass, Ts, traj_control::kGravity, state,
					ref_now, ref_next, _common.limits().tilt_max);

	traj_control::Command command;
	command.thrust = out.thrust;
	command.euler_accel = matrix::Vector3f(out.euler_accel);

	const traj_control::Setpoints setpoints = _common.compute(in.euler, in.euler_rate, command);
	_common.publishSetpoints(setpoints, now, in.timestamp_sample);

	float q_d[4];
	matrix::Quatf(matrix::Eulerf(out.phi_ref, out.theta_ref, in.ref_yaw)).copyTo(q_d);
	_common.publishStatus(traj_control_status_s::CONTROLLER_LAB, true, now, in.reference, in.local_position, in.euler(2),
			      setpoints, out.tilt_saturated, q_d);
}

int LabControl::task_spawn(int argc, char *argv[])
{
	LabControl *instance = new LabControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int LabControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LabControl::print_status()
{
	PX4_INFO("nav_state: %d, active: %d, Ts: %.3f s, hover thrust: %.3f", _nav_state, _was_active,
		 (double)_common.samplePeriod(), (double)_common.model().hover_thrust);

	if (_last_input_failure != 0) {
		PX4_INFO("last input failure (missing, stale or invalid state/reference): %.1f s ago",
			 (double)((hrt_absolute_time() - _last_input_failure) * 1e-6f));
	}

	return PX4_OK;
}

int LabControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
LAB (Linear Algebra Based) trajectory-tracking flight mode. While the LAB mode is active it tracks the reference
published by `traj_generator` with the discrete LAB law and publishes thrust and torque setpoints directly to the
control allocator (PX4's attitude and rate loops are bypassed). It publishes nothing in any other mode.

Gains are `LAB_K1`..`LAB_K12`; the model and limits are the shared `TCC_*` parameters.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("lab_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int lab_control_main(int argc, char *argv[])
{
	return LabControl::main(argc, argv);
}

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
 * @file FlControl.cpp
 */

#include "FlControl.hpp"

#include <lib/mathlib/mathlib.h>

using namespace time_literals;

FlControl::FlControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

bool FlControl::init()
{
	schedule();
	return true;
}

void FlControl::schedule()
{
	_interval_us = math::max(static_cast<uint32_t>(_common.samplePeriod() * 1e6f), static_cast<uint32_t>(1000));
	ScheduleOnInterval(_interval_us);
}

void FlControl::updateParams()
{
	ModuleParams::updateParams();
}

fl_control::Gains FlControl::gains() const
{
	fl_control::Gains g;
	const float values[3][4] = {
		{_param_k11.get(), _param_k12.get(), _param_k13.get(), _param_k14.get()},
		{_param_k21.get(), _param_k22.get(), _param_k23.get(), _param_k24.get()},
		{_param_k31.get(), _param_k32.get(), _param_k33.get(), _param_k34.get()},
	};

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			g.k[i][j] = values[i][j];
		}
	}

	g.k_yaw[0] = _param_k41.get();
	g.k_yaw[1] = _param_k42.get();
	g.exact_fz = _param_exact_fz.get() != 0;
	g.model_derivatives = _param_model_deriv.get() != 0;
	g.heading_compensation = _common.headingCompensation();
	return g;
}

void FlControl::Run()
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

	if (_nav_state != vehicle_status_s::NAVIGATION_STATE_FL) {
		_was_active = false;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float weight = _common.model().mass * traj_control::kGravity;

	if (!_was_active) {
		_common.latchHoverThrust();
		// Entering FL: the thrust integrator starts at hover (u = m g, u_dot = 0), like the paper's initial condition
		_law.reset(weight);
		_common.resetSaturationCount();
		_singular_count = 0;
		_was_active = true;
	}

	traj_control::Inputs in;

	if (!_common.readInputs(now, in)) {
		// Mode active but inputs missing or stale: keep publishing a neutral setpoint until the safety logic leaves the mode
		_common.checkSafety(now, nullptr, false);
		_common.publishNeutral(now);
		_last_input_failure = now;
		return;
	}

	fl_control::State state;
	fl_control::Reference ref;

	for (int i = 0; i < 3; i++) {
		state.position[i] = in.position(i);
		state.velocity[i] = in.velocity(i);
		state.euler[i] = in.euler(i);
		state.euler_rate[i] = in.euler_rate(i);
		ref.position[i] = in.ref_position(i);
		ref.velocity[i] = in.ref_velocity(i);
	}

	ref.yaw = in.ref_yaw;
	ref.yaw_rate = in.ref_yaw_rate;

	const float Ts = _common.samplePeriod();
	const fl_control::Output out = _law.step(gains(), _common.model().mass, Ts, _param_tau.get(), state, ref);

	if (out.singular) {
		_singular_count++;
	}

	// Anti-windup: keep the thrust integrator inside the collective thrust limits (converted to newtons)
	const float to_newton = weight / math::max(_common.model().hover_thrust, 0.05f);
	_law.limitThrust(_common.limits().thrust_min * to_newton, _common.limits().thrust_max * to_newton);

	traj_control::Command command;
	command.thrust = out.thrust;
	command.euler_accel = matrix::Vector3f(out.euler_accel);

	const traj_control::Setpoints setpoints = _common.compute(in.euler, in.euler_rate, command);
	_common.checkSafety(now, &in, !out.singular && PX4_ISFINITE(setpoints.thrust_norm) && setpoints.torque.isAllFinite());
	_common.publishSetpoints(setpoints, now, in.timestamp_sample);

	// FL has no explicit attitude reference: q_d is logged as NaN
	_common.publishStatus(traj_control_status_s::CONTROLLER_FL, true, now, in.reference, in.local_position, in.euler(2),
			      setpoints, false, nullptr);
}

int FlControl::task_spawn(int argc, char *argv[])
{
	FlControl *instance = new FlControl();

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

int FlControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlControl::print_status()
{
	PX4_INFO("nav_state: %d, active: %d, Ts: %.3f s, hover thrust: %.3f, thrust state u: %.2f N, u_dot: %.2f N/s, singular steps: %u",
		 _nav_state, _was_active, (double)_common.samplePeriod(), (double)_common.model().hover_thrust,
		 (double)_law.thrust(), (double)_law.thrustRate(), static_cast<unsigned>(_singular_count));

	if (_last_input_failure != 0) {
		PX4_INFO("last input failure (missing, stale or invalid state/reference): %.1f s ago",
			 (double)((hrt_absolute_time() - _last_input_failure) * 1e-6f));
	}

	return PX4_OK;
}

int FlControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
FL (feedback linearization) trajectory-tracking flight mode. While the FL mode is active it tracks the reference
published by `traj_generator` with the FL law (dynamic extension of the thrust, decoupling matrix and pole placement
gains) and publishes thrust and torque setpoints directly to the control allocator (PX4's attitude and rate loops
are bypassed). It publishes nothing in any other mode.

Gains are `FL_K11`..`FL_K42`; the model and limits are the shared `TCC_*` parameters.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fl_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fl_control_main(int argc, char *argv[])
{
	return FlControl::main(argc, argv);
}

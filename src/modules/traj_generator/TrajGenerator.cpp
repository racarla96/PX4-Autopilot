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
 * @file TrajGenerator.cpp
 */

#include "TrajGenerator.hpp"

#include <lib/mathlib/mathlib.h>

using namespace traj_generator;

TrajGenerator::TrajGenerator(bool test_mode) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_test_mode(test_mode)
{
	updateParams();
}

bool TrajGenerator::init()
{
	ScheduleOnInterval(kIntervalUs);
	return true;
}

void TrajGenerator::updateParams()
{
	ModuleParams::updateParams();
}

bool TrajGenerator::isTrajectoryMode(uint8_t nav_state) const
{
	return nav_state == vehicle_status_s::NAVIGATION_STATE_LAB
	       || nav_state == vehicle_status_s::NAVIGATION_STATE_FL;
}

void TrajGenerator::latchParams()
{
	// Parameters are latched at trajectory start so that every run is reproducible
	// and a parameter change in flight cannot create a jump in the reference.
	_profile_params.type = static_cast<TrajectoryType>(math::constrain<int32_t>(_param_trj_type.get(), 0, 4));
	_profile_params.amplitude = _param_trj_amp.get();
	_profile_params.frequency = _param_trj_freq.get();
	_profile_params.altitude = _param_trj_alt.get();
	_profile_params.altitude_time = _param_trj_alt_t.get();
	_profile_params.duration = _param_trj_duration.get();
	_profile_params.ramp = _param_trj_ramp.get();
}

bool TrajGenerator::tryStart(const hrt_abstime &now)
{
	vehicle_local_position_s lpos{};

	if (!_vehicle_local_position_sub.copy(&lpos) || !lpos.xy_valid || !lpos.z_valid || !lpos.heading_good_for_control
	    || (now - lpos.timestamp) > 500_ms) {
		return false;
	}

	latchParams();
	_profile_start.x = lpos.x;
	_profile_start.y = lpos.y;
	_profile_start.z = lpos.z;
	_profile_start.yaw = lpos.heading;
	_timestamp_start = now;
	_finished = false;
	_started = true;
	return true;
}

void TrajGenerator::publish(const hrt_abstime &now, const ProfileReference &ref, float t, bool valid)
{
	trajectory_reference_s msg{};
	msg.timestamp = now;
	msg.timestamp_start = _timestamp_start;
	msg.t = t;

	for (int i = 0; i < 3; i++) {
		msg.position[i] = ref.position[i];
		msg.velocity[i] = ref.velocity[i];
		msg.acceleration[i] = ref.acceleration[i];
		msg.jerk[i] = ref.jerk[i];
	}

	msg.yaw = ref.yaw;
	msg.yaw_rate = ref.yaw_rate;
	msg.valid = valid;
	msg.finished = ref.finished;
	_trajectory_reference_pub.publish(msg);
}

void TrajGenerator::Run()
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
	}

	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		_nav_state = status.nav_state;
	}

	const hrt_abstime now = hrt_absolute_time();
	const bool active = _test_mode || isTrajectoryMode(_nav_state);

	if (!active) {
		if (_was_active) {
			// Tell the controllers that the reference is gone
			publish(now, ProfileReference{}, 0.f, false);
		}

		_was_active = false;
		_started = false;
		_last_nav_state = _nav_state;
		return;
	}

	// Restart on every transition into LAB or FL, including LAB <-> FL
	if (!_was_active || _restart_requested || (!_test_mode && _nav_state != _last_nav_state)) {
		_started = false;
		_restart_requested = false;
	}

	_was_active = true;
	_last_nav_state = _nav_state;

	if (!_started && !tryStart(now)) {
		publish(now, ProfileReference{}, 0.f, false);
		return;
	}

	const float t = static_cast<float>(now - _timestamp_start) * 1e-6f;
	const ProfileReference ref = evaluate(_profile_params, _profile_start, t);
	_finished = ref.finished;
	publish(now, ref, t, true);
}

int TrajGenerator::task_spawn(int argc, char *argv[])
{
	bool test_mode = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "t", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 't':
			test_mode = true;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	TrajGenerator *instance = new TrajGenerator(test_mode);

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

int TrajGenerator::custom_command(int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "restart")) {
		if (!is_running()) {
			return print_usage("not running");
		}

		get_instance()->requestRestart();
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int TrajGenerator::print_status()
{
	PX4_INFO("mode: %s, nav_state: %d, active: %d, started: %d, finished: %d",
		 _test_mode ? "test (free running)" : "LAB/FL", _nav_state, _was_active, _started, _finished);

	if (_started) {
		PX4_INFO("type: %d, amplitude: %.2f m, frequency: %.3f Hz, altitude: %.1f m, duration: %.1f s, ramp: %.1f s",
			 static_cast<int>(_profile_params.type), (double)_profile_params.amplitude, (double)_profile_params.frequency,
			 (double)_profile_params.altitude, (double)_profile_params.duration, (double)_profile_params.ramp);
		PX4_INFO("start NED: [%.2f, %.2f, %.2f] m, yaw %.2f rad", (double)_profile_start.x, (double)_profile_start.y,
			 (double)_profile_start.z, (double)_profile_start.yaw);
	}

	return PX4_OK;
}

int TrajGenerator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Generates the reference trajectory (hover, circle or figure-8) tracked by the LAB and FL flight modes
and publishes it on `trajectory_reference` at 100 Hz.

The trajectory starts at the vehicle position when LAB or FL is entered, and its clock restarts on every
transition into either mode, so repeated runs are comparable. Parameters `TRJ_*` are latched at that moment.

### Examples
Run free (without entering LAB/FL) and restart the trajectory clock:
$ traj_generator start -t
$ traj_generator restart
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("traj_generator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Test mode: run without LAB/FL being the active mode", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("restart", "Restart the trajectory clock");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int traj_generator_main(int argc, char *argv[])
{
	return TrajGenerator::main(argc, argv);
}

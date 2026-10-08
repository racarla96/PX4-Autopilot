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
 * @file TrajGenerator.hpp
 *
 * Publishes the reference trajectory shared by the LAB and FL flight modes.
 * The trajectory clock restarts on every transition into LAB or FL (including LAB <-> FL),
 * and the start state is captured from the local position estimate at that moment.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_reference.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "TrajectoryProfile.hpp"

using namespace time_literals;

class TrajGenerator : public ModuleBase<TrajGenerator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	/** @param test_mode run without LAB/FL being the active mode (bench/SITL checks) */
	explicit TrajGenerator(bool test_mode);
	~TrajGenerator() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** Restart the trajectory clock at the next cycle (test mode command). */
	void requestRestart() { _restart_requested = true; }

private:
	static constexpr uint32_t kIntervalUs = 10_ms; // 100 Hz

	void Run() override;
	void updateParams() override;

	bool isTrajectoryMode(uint8_t nav_state) const;
	void latchParams();
	bool tryStart(const hrt_abstime &now);
	void publish(const hrt_abstime &now, const traj_generator::ProfileReference &ref, float t, bool valid);

	uORB::Publication<trajectory_reference_s> _trajectory_reference_pub{ORB_ID(trajectory_reference)};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	traj_generator::ProfileParams _profile_params{};
	traj_generator::ProfileStart _profile_start{};

	const bool _test_mode;

	uint8_t _nav_state{vehicle_status_s::NAVIGATION_STATE_MANUAL};
	uint8_t _last_nav_state{vehicle_status_s::NAVIGATION_STATE_MANUAL};
	bool _was_active{false};
	bool _restart_requested{false};
	bool _started{false};
	hrt_abstime _timestamp_start{0};
	bool _finished{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::TRJ_TYPE>) _param_trj_type,
		(ParamFloat<px4::params::TRJ_AMP>) _param_trj_amp,
		(ParamFloat<px4::params::TRJ_FREQ>) _param_trj_freq,
		(ParamFloat<px4::params::TRJ_ALT>) _param_trj_alt,
		(ParamFloat<px4::params::TRJ_ALT_T>) _param_trj_alt_t,
		(ParamFloat<px4::params::TRJ_DURATION>) _param_trj_duration,
		(ParamFloat<px4::params::TRJ_RAMP>) _param_trj_ramp
	)
};
